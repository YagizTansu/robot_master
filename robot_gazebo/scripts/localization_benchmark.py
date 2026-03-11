#!/usr/bin/env python3
"""
Localization Benchmarking Node
================================
Compares any localization algorithm output against the Gazebo ground truth.

Subscribed Topics:
  /ground_truth/odom          (nav_msgs/Odometry)  — 100% accurate Gazebo pose
  <est_topic>                 (nav_msgs/Odometry)  — algorithm under test
                               default: /fgo/odometry

Published Topics:
  /benchmark/position_error   (std_msgs/Float64)   — |p_est - p_gt|  [m]
  /benchmark/yaw_error_deg    (std_msgs/Float64)   — |yaw_est - yaw_gt| [deg]
  /benchmark/ate_rmse         (std_msgs/Float64)   — running ATE RMSE [m]
  /benchmark/gt_path          (nav_msgs/Path)       — ground truth trajectory
  /benchmark/est_path         (nav_msgs/Path)       — estimated trajectory
                               (both for RViz visualisation)

On node shutdown a CSV report is written to /tmp/localization_benchmark_<algo>.csv
Each row: timestamp, x_gt, y_gt, yaw_gt_deg, x_est, y_est, yaw_est_deg,
          pos_err_m, yaw_err_deg

Usage:
  ros2 run robot_gazebo localization_benchmark.py
  ros2 run robot_gazebo localization_benchmark.py --ros-args \
      -p estimated_topic:=/amcl_pose_as_odom \
      -p algorithm_name:=AMCL \
      -p csv_output_dir:=/tmp
"""

import os
import csv
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import message_filters
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


# ── Helpers ───────────────────────────────────────────────────────────────────

def quat_to_yaw(q) -> float:
    """Convert geometry_msgs/Quaternion to yaw angle (radians)."""
    # Standard Z-Y-X Euler extraction for yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


# ── Node ──────────────────────────────────────────────────────────────────────

class LocalizationBenchmark(Node):
    """
    Synchronises ground truth and estimated odometry messages (approx. time sync),
    computes error metrics, and accumulates trajectory data for ATE and CSV export.
    """

    def __init__(self) -> None:
        super().__init__("localization_benchmark")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("estimated_topic", "/fgo/odometry")
        self.declare_parameter("algorithm_name",  "FGO")
        self.declare_parameter("csv_output_dir",  "/tmp")
        self.declare_parameter("path_max_len",    5000)   # max poses kept in Path msg
        self.declare_parameter("sync_slop_sec",   0.15)   # ApproxTime tolerance [s]

        self._est_topic   = self.get_parameter("estimated_topic").get_parameter_value().string_value
        self._algo_name   = self.get_parameter("algorithm_name").get_parameter_value().string_value
        self._csv_dir     = self.get_parameter("csv_output_dir").get_parameter_value().string_value
        self._path_maxlen = self.get_parameter("path_max_len").get_parameter_value().integer_value
        self._sync_slop   = self.get_parameter("sync_slop_sec").get_parameter_value().double_value

        # ── State ──────────────────────────────────────────────────────────
        self._records: list[dict] = []           # for CSV + ATE
        self._sum_sq_err: float  = 0.0           # running Σ(pos_err²)
        self._n_samples: int     = 0

        # ── Subscribers (message_filters for time sync) ─────────────────────
        gt_qos  = QoSProfile(depth=50,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)
        est_qos = QoSProfile(depth=50,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)

        self._sub_gt  = message_filters.Subscriber(self, Odometry,
                                                    "/ground_truth/odom",
                                                    qos_profile=gt_qos)
        self._sub_est = message_filters.Subscriber(self, Odometry,
                                                    self._est_topic,
                                                    qos_profile=est_qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_gt, self._sub_est],
            queue_size=50,
            slop=self._sync_slop,
        )
        self._sync.registerCallback(self._on_sync)

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_pos_err  = self.create_publisher(Float64, "/benchmark/position_error",  10)
        self._pub_yaw_err  = self.create_publisher(Float64, "/benchmark/yaw_error_deg",   10)
        self._pub_ate      = self.create_publisher(Float64, "/benchmark/ate_rmse",        10)
        self._pub_gt_path  = self.create_publisher(Path,    "/benchmark/gt_path",         10)
        self._pub_est_path = self.create_publisher(Path,    "/benchmark/est_path",        10)

        # ── Path accumulators ───────────────────────────────────────────────
        self._gt_path  = Path()
        self._est_path = Path()
        self._gt_path.header.frame_id  = "map"
        self._est_path.header.frame_id = "map"

        # ── Periodic status log ─────────────────────────────────────────────
        self._log_timer = self.create_timer(10.0, self._log_status)

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  Localization Benchmark — {self._algo_name}\n"
            f"{'='*60}\n"
            f"  Ground truth  : /ground_truth/odom\n"
            f"  Estimated     : {self._est_topic}\n"
            f"  Sync slop     : {self._sync_slop} s\n"
            f"  CSV output    : {self._csv_dir}/localization_benchmark_{self._algo_name}.csv\n"
            f"{'='*60}"
        )

    # ── Sync callback ─────────────────────────────────────────────────────────

    def _on_sync(self, gt_msg: Odometry, est_msg: Odometry) -> None:
        """Called whenever a ground-truth and estimated message are time-matched."""
        stamp = gt_msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        # Extract positions
        xg = gt_msg.pose.pose.position.x
        yg = gt_msg.pose.pose.position.y
        xe = est_msg.pose.pose.position.x
        ye = est_msg.pose.pose.position.y

        # Extract yaws
        yaw_gt  = quat_to_yaw(gt_msg.pose.pose.orientation)
        yaw_est = quat_to_yaw(est_msg.pose.pose.orientation)

        # ── Error metrics ────────────────────────────────────────────────────
        dx = xe - xg
        dy = ye - yg
        pos_err_m    = math.sqrt(dx * dx + dy * dy)
        yaw_err_rad  = abs(wrap_angle(yaw_est - yaw_gt))
        yaw_err_deg  = math.degrees(yaw_err_rad)

        # ── Running ATE (RMSE of position error) ────────────────────────────
        self._sum_sq_err += pos_err_m ** 2
        self._n_samples  += 1
        ate_rmse = math.sqrt(self._sum_sq_err / self._n_samples)

        # ── Publish metrics ──────────────────────────────────────────────────
        self._pub_pos_err.publish(Float64(data=pos_err_m))
        self._pub_yaw_err.publish(Float64(data=yaw_err_deg))
        self._pub_ate.publish(Float64(data=ate_rmse))

        # ── Accumulate paths ─────────────────────────────────────────────────
        self._append_path(self._gt_path,  gt_msg,  stamp)
        self._append_path(self._est_path, est_msg, stamp)
        self._pub_gt_path.publish(self._gt_path)
        self._pub_est_path.publish(self._est_path)

        # ── Record for CSV ───────────────────────────────────────────────────
        self._records.append({
            "timestamp":    t,
            "x_gt":         xg,
            "y_gt":         yg,
            "yaw_gt_deg":   math.degrees(yaw_gt),
            "x_est":        xe,
            "y_est":        ye,
            "yaw_est_deg":  math.degrees(yaw_est),
            "pos_err_m":    pos_err_m,
            "yaw_err_deg":  yaw_err_deg,
            "ate_rmse_m":   ate_rmse,
        })

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _append_path(self, path: Path, odom: Odometry, stamp) -> None:
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = "map"
        ps.pose            = odom.pose.pose
        path.header.stamp  = stamp
        path.poses.append(ps)
        # Trim to max length to avoid unbounded memory growth
        if len(path.poses) > self._path_maxlen:
            path.poses = path.poses[-self._path_maxlen:]

    def _log_status(self) -> None:
        if self._n_samples == 0:
            self.get_logger().warn(
                f"[{self._algo_name}] No synchronised pairs yet.\n"
                f"  Make sure both /ground_truth/odom and {self._est_topic} are active."
            )
            return

        ate = math.sqrt(self._sum_sq_err / self._n_samples)
        last = self._records[-1]
        self.get_logger().info(
            f"[{self._algo_name}] n={self._n_samples:5d} | "
            f"pos_err={last['pos_err_m']:.3f} m | "
            f"yaw_err={last['yaw_err_deg']:.2f}° | "
            f"ATE RMSE={ate:.3f} m"
        )

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def save_csv(self) -> None:
        if not self._records:
            self.get_logger().warn("No data collected – CSV not written.")
            return

        fname = os.path.join(
            self._csv_dir,
            f"localization_benchmark_{self._algo_name}.csv"
        )
        fieldnames = [
            "timestamp",
            "x_gt", "y_gt", "yaw_gt_deg",
            "x_est", "y_est", "yaw_est_deg",
            "pos_err_m", "yaw_err_deg", "ate_rmse_m",
        ]
        with open(fname, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._records)

        # Summary stats
        errors = [r["pos_err_m"] for r in self._records]
        ate    = math.sqrt(sum(e**2 for e in errors) / len(errors))
        max_e  = max(errors)
        mean_e = sum(errors) / len(errors)

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  BENCHMARK SUMMARY — {self._algo_name}\n"
            f"{'='*60}\n"
            f"  Samples     : {len(self._records)}\n"
            f"  ATE RMSE    : {ate:.4f} m\n"
            f"  Mean error  : {mean_e:.4f} m\n"
            f"  Max error   : {max_e:.4f} m\n"
            f"  CSV saved   : {fname}\n"
            f"{'='*60}"
        )


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationBenchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
