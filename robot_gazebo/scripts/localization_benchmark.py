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
import curses
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import message_filters
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


# ── Dashboard colour thresholds ───────────────────────────────────────────────
_POS_GOOD, _POS_WARN = 0.05, 0.15   # metres
_YAW_GOOD, _YAW_WARN = 2.0,  5.0    # degrees
_ATE_GOOD, _ATE_WARN = 0.05, 0.20   # metres


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
        self.declare_parameter("dashboard",        True)   # set False for headless/CI

        self._est_topic   = self.get_parameter("estimated_topic").get_parameter_value().string_value
        self._algo_name   = self.get_parameter("algorithm_name").get_parameter_value().string_value
        self._csv_dir     = self.get_parameter("csv_output_dir").get_parameter_value().string_value
        self._path_maxlen = self.get_parameter("path_max_len").get_parameter_value().integer_value
        self._sync_slop   = self.get_parameter("sync_slop_sec").get_parameter_value().double_value
        self.dashboard    = self.get_parameter("dashboard").get_parameter_value().bool_value

        # ── Live metric state (read by dashboard draw loop) ─────────────────
        self.pos_err:   float = float("nan")
        self.yaw_err:   float = float("nan")
        self.ate_rmse:  float = float("nan")
        self.last_update: float = 0.0   # monotonic time of last sync pair

        # ── State ──────────────────────────────────────────────────────────
        self._records: list[dict] = []           # for CSV + ATE
        self._sum_sq_err: float  = 0.0           # running Σ(pos_err²) — calibrated
        self._n_samples: int     = 0

        # ── First-sample calibration ────────────────────────────────────────
        # The Gazebo world frame and the ROS map frame share the same frame_id
        # ("map"), but their origins may differ by a constant offset because
        # the slam_toolbox map was built with the robot at a non-zero Gazebo
        # world position.  The result: GT reports (0, 0) at spawn while FGO
        # (after NDT correction) reports the true map-frame position, showing
        # a spurious constant error in the benchmark.
        #
        # We absorb this by recording the initial FGO-vs-GT residual on the
        # VERY FIRST synchronised pair and subtracting it from all subsequent
        # residuals.  This converts the benchmark from measuring "absolute
        # position in potentially-misaligned frames" to measuring "how well
        # FGO tracks ground-truth motion relative to the starting pose" — which
        # is the actually meaningful localisation quality metric.
        #
        # The uncalibrated (raw) error is also stored in the CSV so the frame
        # offset can be read off from the first few rows and, if desired, be
        # applied to the ground_truth_publisher's world_to_map_offset_x/y
        # parameters to permanently align the two frames.
        self._calib_dx:   float | None = None   # (x_est - x_gt) at t=0
        self._calib_dy:   float | None = None   # (y_est - y_gt) at t=0
        self._calib_dyaw: float | None = None   # wrap(yaw_est - yaw_gt) at t=0

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
        dx_raw = xe - xg
        dy_raw = ye - yg
        raw_pos_err_m   = math.sqrt(dx_raw * dx_raw + dy_raw * dy_raw)
        raw_yaw_err_rad = abs(wrap_angle(yaw_est - yaw_gt))
        raw_yaw_err_deg = math.degrees(raw_yaw_err_rad)

        # ── First-pair calibration ────────────────────────────────────────
        # Absorb the constant world→map frame offset into the baseline so
        # the reported error reflects FGO's tracking quality, not the
        # accidental alignment of the two starting positions.
        if self._calib_dx is None:
            self._calib_dx   = dx_raw
            self._calib_dy   = dy_raw
            self._calib_dyaw = wrap_angle(yaw_est - yaw_gt)
            self.get_logger().info(
                f"[{self._algo_name}] Frame calibration captured: "
                f"Δx={self._calib_dx:.4f} m  Δy={self._calib_dy:.4f} m  "
                f"Δyaw={math.degrees(self._calib_dyaw):.2f}°\n"
                f"  (non-zero = Gazebo world ≠ ROS map frame origin; "
                f"set world_to_map_offset_x/y in ground_truth_publisher to fix permanently)"
            )

        dx = dx_raw - self._calib_dx
        dy = dy_raw - self._calib_dy
        pos_err_m    = math.sqrt(dx * dx + dy * dy)
        yaw_err_rad  = abs(wrap_angle((yaw_est - yaw_gt) - self._calib_dyaw))
        yaw_err_deg  = math.degrees(yaw_err_rad)

        # ── Running ATE (RMSE of position error) ────────────────────────────
        self._sum_sq_err += pos_err_m ** 2
        self._n_samples  += 1
        ate_rmse = math.sqrt(self._sum_sq_err / self._n_samples)

        # ── Update live dashboard state ──────────────────────────────────────
        self.pos_err    = pos_err_m
        self.yaw_err    = yaw_err_deg
        self.ate_rmse   = ate_rmse
        self.last_update = time.monotonic()

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
            "timestamp":        t,
            "x_gt":             xg,
            "y_gt":             yg,
            "yaw_gt_deg":       math.degrees(yaw_gt),
            "x_est":            xe,
            "y_est":            ye,
            "yaw_est_deg":      math.degrees(yaw_est),
            # Calibrated (frame-offset-corrected) metrics — use these for analysis
            "pos_err_m":        pos_err_m,
            "yaw_err_deg":      yaw_err_deg,
            "ate_rmse_m":       ate_rmse,
            # Raw (uncalibrated) metrics — first few rows reveal the frame offset
            "raw_pos_err_m":    raw_pos_err_m,
            "raw_yaw_err_deg":  raw_yaw_err_deg,
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
            f"pos_err={last['pos_err_m']:.3f} m (cal) | "
            f"raw={last['raw_pos_err_m']:.3f} m | "
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
            # calibrated (frame-offset-corrected) — primary metrics
            "pos_err_m", "yaw_err_deg", "ate_rmse_m",
            # raw (not frame-corrected) — useful for determining the offset value
            "raw_pos_err_m", "raw_yaw_err_deg",
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
            f"  ATE RMSE    : {ate:.4f} m  (calibrated)\n"
            f"  Mean error  : {mean_e:.4f} m  (calibrated)\n"
            f"  Max error   : {max_e:.4f} m  (calibrated)\n"
            f"  Frame offset: Δx={self._calib_dx:.4f} m  Δy={self._calib_dy:.4f} m\n"
            f"  CSV saved   : {fname}\n"
            f"{'='*60}"
        )


# ── Dashboard helpers ─────────────────────────────────────────────────────────

def _colour(value: float, good: float, warn: float) -> int:
    if math.isnan(value):
        return curses.color_pair(4)
    if value <= good:
        return curses.color_pair(1)   # green
    if value <= warn:
        return curses.color_pair(2)   # yellow
    return curses.color_pair(3)       # red


def _bar(value: float, max_val: float, width: int = 16) -> str:
    if math.isnan(value) or max_val <= 0:
        return "[" + "?" * width + "]"
    filled = int(min(value / max_val, 1.0) * width)
    return "[" + "█" * filled + "░" * (width - filled) + "]"


def _draw(stdscr, node: LocalizationBenchmark, stop_event: threading.Event) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN,  -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_RED,    -1)
    curses.init_pair(4, curses.COLOR_WHITE,  -1)
    curses.init_pair(5, curses.COLOR_CYAN,   -1)

    TITLE = curses.A_BOLD | curses.color_pair(5)
    BOLD  = curses.A_BOLD
    DIM   = curses.color_pair(4)

    while not stop_event.is_set():
        ch = stdscr.getch()
        if ch in (ord("q"), ord("Q")):
            stop_event.set()
            break

        stdscr.erase()
        h, w = stdscr.getmaxyx()
        box_w = min(58, w - 2)
        bx    = (w - box_w) // 2
        by    = max(0, (h - 14) // 2)
        sep   = "─" * (box_w - 2)

        def put(row, col, txt, attr=curses.A_NORMAL):
            try:
                stdscr.addstr(by + row, bx + col, txt, attr)
            except curses.error:
                pass

        # border
        put(0,  0, "╔" + sep + "╗", TITLE)
        title = f"  BENCHMARK  ─  {node._algo_name}  "
        put(1,  0, "║" + title.center(box_w - 2) + "║", TITLE)
        put(2,  0, "╠" + sep + "╣", TITLE)

        # metric rows
        for row, label, val, unit, dec, gd, wd, mx in [
            (3, "Position Error", node.pos_err,  "m",  3, _POS_GOOD, _POS_WARN, 0.30),
            (4, "Yaw Error",      node.yaw_err,  "°",  2, _YAW_GOOD, _YAW_WARN, 10.0),
            (5, "ATE RMSE",       node.ate_rmse, "m",  4, _ATE_GOOD, _ATE_WARN, 0.30),
        ]:
            col_attr = _colour(val, gd, wd)
            val_str  = f"{'—':>8}" if math.isnan(val) else f"{val:>8.{dec}f}"
            bar      = _bar(val, mx)
            put(row, 0, "║", TITLE)
            put(row, 2, f"{label:<18}", BOLD)
            put(row, 20, f"{val_str} {unit:<2}", col_attr | BOLD)
            put(row, 32, f"  {bar}", col_attr)
            put(row, box_w - 1, "║", TITLE)

        put(6, 0, "╠" + sep + "╣", TITLE)

        # samples + last update
        put(7, 0, "║", TITLE)
        put(7, 2, f"Samples : {node._n_samples:>8}", BOLD)
        put(7, box_w - 1, "║", TITLE)

        put(8, 0, "║", TITLE)
        if node.last_update > 0.0:
            age = time.monotonic() - node.last_update
            age_s   = f"{age:.1f} s ago"
            age_atr = curses.color_pair(1) if age < 1.0 else \
                      curses.color_pair(2) if age < 5.0 else \
                      curses.color_pair(3)
        else:
            age_s, age_atr = "waiting for data…", DIM
        put(8, 2, "Last msg: ", BOLD)
        put(8, 12, f"{age_s:<{box_w - 15}}", age_atr)
        put(8, box_w - 1, "║", TITLE)

        put(9,  0, "╠" + sep + "╣", TITLE)
        put(10, 0, "║", TITLE)
        put(10, 3,  "● Good ", curses.color_pair(1) | BOLD)
        put(10, 11, "● Warn ", curses.color_pair(2) | BOLD)
        put(10, 19, "● Bad  ", curses.color_pair(3) | BOLD)
        put(10, 30, "press Q to quit", DIM)
        put(10, box_w - 1, "║", TITLE)
        put(11, 0, "╚" + sep + "╝", TITLE)

        stdscr.refresh()
        time.sleep(0.1)   # 10 Hz


# ── Main ──────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationBenchmark()

    stop_event = threading.Event()

    # ROS spin in background thread
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        if node.dashboard:
            curses.wrapper(_draw, node, stop_event)
        else:
            # Headless: block until Ctrl-C
            spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        node.save_csv()
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
