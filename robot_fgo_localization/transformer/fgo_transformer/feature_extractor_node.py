#!/usr/bin/env python3
"""
Feature Extractor Node
======================
Collects raw sensor signals, FGO estimates, and Gazebo ground truth poses,
computes a per-timestep feature vector, and writes everything to a CSV file.

The CSV is the training dataset for the Transformer trust-weight model.
Each row is written whenever a /ground_truth/odom + /fgo/odometry pair is
time-matched by the ApproximateTimeSynchronizer (~10–50 Hz in practice).

CSV columns
-----------
timestamp_sec     — ROS sim time (float seconds)

--- LiDAR features ---
fitness_score     — NDT scan-match fitness score (lower = better match quality)
fitness_age_sec   — seconds since last fitness_score message (staleness marker)

--- IMU features ---
accel_norm_dev    — |‖[ax,ay,az]‖ - g|  vibration / shock proxy (m/s²)
gyro_z            — z-axis angular rate from IMU (rad/s)
accel_norm_std    — std-dev of ‖accel‖ over last N IMU samples (vibration envelope)
gyro_z_std        — std-dev of gyro_z over last N IMU samples (rotation flutter)

--- Encoder / odometry features ---
linear_vel        — wheel-odometry linear velocity  vx (m/s)
angular_vel       — wheel-odometry angular velocity wz (rad/s)
jerk              — |Δvx / Δt|  sudden-slip proxy (m/s²)
slip_metric       — |gyro_z - angular_vel|  encoder-IMU angular mismatch (rad/s)

--- Ground truth labels (TRAINING TARGETS) ---
gt_x, gt_y        — Gazebo ground truth position (m)
gt_yaw_rad        — Gazebo ground truth yaw (rad)
fgo_x, fgo_y      — FGO estimated position (m)
fgo_yaw_rad       — FGO estimated yaw (rad)
pos_err_m         — ‖(fgo - gt)‖  PRIMARY TRAINING LABEL (m)
yaw_err_rad       — |wrap(fgo_yaw - gt_yaw)|  supplementary label (rad)

--- Per-sensor GT trust labels (derived from Gazebo ground truth) ---
lidar_pos_error   — ‖scan_match_pose - GT_pose‖ (m)  LiDAR measurement error vs GT
encoder_drift     — ‖encoder_dead_reckoned_disp - GT_disp‖ over drift_window (m)
imu_yaw_error     — |integrated_gyro_yaw_change - GT_yaw_change| over drift_window (rad)

These three columns replace rule-based pseudo-labels.  They measure each
sensor's raw measurement quality directly against Gazebo ground truth —
no hand-crafted thresholds required.

Usage
-----
    ros2 launch fgo_transformer feature_extractor.launch.py
"""

import collections
import csv
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


# ─────────────────────────────────────────────────────────────────────────────
# CSV schema — order defines column order in the output file
# ─────────────────────────────────────────────────────────────────────────────

_CSV_COLUMNS = [
    # time
    "timestamp_sec",
    # lidar features
    "fitness_score",
    "fitness_age_sec",
    # imu features
    "accel_norm_dev",
    "gyro_z",
    "accel_norm_std",
    "gyro_z_std",
    # encoder features
    "linear_vel",
    "angular_vel",
    "jerk",
    "slip_metric",
    # ground truth + fgo poses (regression labels)
    "gt_x",
    "gt_y",
    "gt_yaw_rad",
    "fgo_x",
    "fgo_y",
    "fgo_yaw_rad",
    # FGO vs GT error labels
    "pos_err_m",
    "yaw_err_rad",
    # Per-sensor GT trust labels (new — from Gazebo GT, no hand-crafted thresholds)
    "lidar_pos_error",   # ‖scan_match_pose - GT_pose‖ (m)
    "encoder_drift",     # ‖encoder_disp - GT_disp‖ over drift window (m)
    "imu_yaw_error",     # |gyro_integrated_Δyaw - GT_Δyaw| over drift window (rad)
]


# ─────────────────────────────────────────────────────────────────────────────
# Pure helpers (no ROS dependencies)
# ─────────────────────────────────────────────────────────────────────────────

def _quat_to_yaw(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def _pop_std(buf) -> float:
    """Population std-dev of a sequence; returns NaN if fewer than 2 samples."""
    n = len(buf)
    if n < 2:
        return float("nan")
    mean = sum(buf) / n
    return math.sqrt(sum((x - mean) ** 2 for x in buf) / n)


def _fmt(v: float, precision: int = 6) -> str:
    """Format a float for CSV; preserves 'nan' and 'inf' strings."""
    if math.isnan(v):
        return "nan"
    if math.isinf(v):
        return "inf"
    return f"{v:.{precision}f}"


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class FeatureExtractorNode(Node):
    """
    Subscribes to sensor and localization topics, computes a fixed-width
    feature vector at each synchronized (GT, FGO) pair, and streams rows
    to a CSV file for offline Transformer training.

    GT trust labels are derived by comparing each sensor's raw measurement
    to Gazebo ground truth — no hand-crafted thresholds required.
    """

    def __init__(self) -> None:
        super().__init__("feature_extractor")

        # ── Parameters ─────────────────────────────────────────────────────
        _default_data_dir = os.path.join(
            get_package_share_directory("fgo_transformer"), "data")
        self.declare_parameter("csv_output_dir",       _default_data_dir)
        self.declare_parameter("sync_slop_sec",        0.15)
        self.declare_parameter("imu_buffer_size",      50)     # ~0.5 s at 100 Hz
        self.declare_parameter("gravity",              9.81)
        self.declare_parameter("max_fitness_age_sec",  2.0)    # stale → NaN
        self.declare_parameter("max_scan_match_age_sec", 2.0)  # stale lidar pose → NaN
        self.declare_parameter("drift_window_sec",     2.0)    # window for encoder/IMU drift

        _csv_param = str(self.get_parameter("csv_output_dir").value).strip()
        csv_dir = (os.path.expanduser(_csv_param)
                   if _csv_param
                   else os.path.join(
                       get_package_share_directory("fgo_transformer"), "data"))
        sync_slop              = float(self.get_parameter("sync_slop_sec").value)
        imu_buf_size           = int(self.get_parameter("imu_buffer_size").value)
        self._gravity          = float(self.get_parameter("gravity").value)
        self._max_fit_age      = float(self.get_parameter("max_fitness_age_sec").value)
        self._max_scan_age     = float(self.get_parameter("max_scan_match_age_sec").value)
        self._drift_window_sec = float(self.get_parameter("drift_window_sec").value)

        # ── Shared lock for all cached sensor state ─────────────────────────
        # Sync callback takes a snapshot copy; contention is negligible.
        self._lock = threading.Lock()

        # ── IMU rolling buffers ─────────────────────────────────────────────
        self._accel_buf: collections.deque = collections.deque(maxlen=imu_buf_size)
        self._gyro_buf:  collections.deque = collections.deque(maxlen=imu_buf_size)
        self._latest_accel_norm: float = float("nan")
        self._latest_gyro_z:    float = float("nan")

        # IMU yaw integration (for imu_yaw_error label)
        self._imu_yaw_abs:  float = 0.0           # running integral of gyro_z (rad)
        self._prev_imu_sec: float = float("nan")  # timestamp of last IMU sample

        # ── Wheel odometry ──────────────────────────────────────────────────
        self._linear_vel:    float = float("nan")
        self._angular_vel:   float = float("nan")
        self._jerk:          float = float("nan")
        self._prev_lin_vel:  float = float("nan")
        self._prev_odom_sec: float = float("nan")

        # Encoder dead-reckoning (for encoder_drift label)
        # Integrated in the body→world frame using running yaw estimate.
        self._enc_x:   float = 0.0   # integrated encoder x (m)
        self._enc_y:   float = 0.0   # integrated encoder y (m)
        self._enc_yaw: float = 0.0   # integrated encoder yaw (rad)

        # ── NDT fitness score ───────────────────────────────────────────────
        self._fitness_score:    float = float("nan")
        self._fitness_mono_sec: float = float("nan")

        # ── LiDAR scan match pose (for lidar_pos_error label) ──────────────
        # /scan_match_pose carries only ACCEPTED scans (fitness < threshold).
        # If the topic goes stale, lidar_pos_error is written as NaN.
        self._scan_match_x:        float = float("nan")
        self._scan_match_y:        float = float("nan")
        self._scan_match_mono_sec: float = float("nan")

        # ── Drift buffer ────────────────────────────────────────────────────
        # Each entry: (timestamp_sec, gt_x, gt_y, gt_yaw, enc_x, enc_y, imu_yaw_abs)
        # Appended in _on_sync; accessed only from that callback → no locking needed.
        # maxlen keeps at most drift_window * 3 entries (generous for up to 30 Hz sync).
        _drift_maxlen = max(10, int(self._drift_window_sec * 30) + 10)
        self._drift_buf: collections.deque = collections.deque(maxlen=_drift_maxlen)

        # Row counter for logging
        self._row_count: int = 0

        # ── CSV output ──────────────────────────────────────────────────────
        os.makedirs(csv_dir, exist_ok=True)
        ts_str   = time.strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(csv_dir, f"features_{ts_str}.csv")

        # buffering=1 → line-buffered: each row is flushed to disk immediately
        self._csv_file   = open(csv_path, "w", newline="", buffering=1)
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=_CSV_COLUMNS)
        self._csv_writer.writeheader()

        # ── QoS profiles ────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ── Independent (cached) subscribers ────────────────────────────────
        self.create_subscription(Imu,      "/imu",                      self._on_imu,     sensor_qos)
        self.create_subscription(Odometry, "/odometry",                 self._on_odom,    reliable_qos)
        self.create_subscription(Float64,  "/scan_match/fitness_score", self._on_fitness, reliable_qos)

        # Scan match pose — ACCEPTED scans only; used for lidar_pos_error GT label.
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/scan_match_pose",
            self._on_scan_match,
            reliable_qos,
        )

        # ── ApproximateTimeSynchronizer: GT + FGO drive each CSV row ────────
        sub_gt  = message_filters.Subscriber(self, Odometry, "/ground_truth/odom", qos_profile=reliable_qos)
        sub_fgo = message_filters.Subscriber(self, Odometry, "/fgo/odometry",       qos_profile=reliable_qos)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [sub_gt, sub_fgo], queue_size=50, slop=sync_slop)
        self._sync.registerCallback(self._on_sync)

        # ── Periodic status log ──────────────────────────────────────────────
        self._log_timer = self.create_timer(10.0, self._log_status)

        self.get_logger().info(
            f"\n{'=' * 60}\n"
            f"  FeatureExtractorNode started\n"
            f"{'=' * 60}\n"
            f"  CSV output     : {csv_path}\n"
            f"  IMU buffer     : {imu_buf_size} samples (~"
            f"{imu_buf_size / 100.0:.1f} s at 100 Hz)\n"
            f"  Sync slop      : {sync_slop} s\n"
            f"  Gravity        : {self._gravity} m/s²\n"
            f"  Max fit age    : {self._max_fit_age} s\n"
            f"  Max scan age   : {self._max_scan_age} s\n"
            f"  Drift window   : {self._drift_window_sec} s\n"
            f"{'=' * 60}"
        )

    # ── Independent callbacks ──────────────────────────────────────────────────

    def _on_imu(self, msg: Imu) -> None:
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        gz = msg.angular_velocity.z

        # Timestamp for gyro integration dt
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            self._accel_buf.append(accel_norm)
            self._gyro_buf.append(gz)
            self._latest_accel_norm = accel_norm
            self._latest_gyro_z     = gz

            # Integrate gyro_z for IMU yaw error GT label
            if not math.isnan(self._prev_imu_sec):
                dt = sec - self._prev_imu_sec
                if 0.0 < dt < 0.5:           # sanity: reject large gaps or backwards time
                    self._imu_yaw_abs += gz * dt
            self._prev_imu_sec = sec

    def _on_odom(self, msg: Odometry) -> None:
        vx  = msg.twist.twist.linear.x
        wz  = msg.twist.twist.angular.z
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            if (not math.isnan(self._prev_lin_vel)
                    and not math.isnan(self._prev_odom_sec)):
                dt = sec - self._prev_odom_sec
                if dt > 1e-6:
                    self._jerk = abs(vx - self._prev_lin_vel) / dt
                    # ── Encoder dead-reckoning integration ────────────────
                    # Use midpoint yaw for slightly better accuracy.
                    new_enc_yaw = self._enc_yaw + wz * dt
                    mid_yaw     = (self._enc_yaw + new_enc_yaw) * 0.5
                    self._enc_x   += vx * math.cos(mid_yaw) * dt
                    self._enc_y   += vx * math.sin(mid_yaw) * dt
                    self._enc_yaw  = new_enc_yaw
                else:
                    self._jerk = 0.0
            else:
                self._jerk = float("nan")

            self._linear_vel    = vx
            self._angular_vel   = wz
            self._prev_lin_vel  = vx
            self._prev_odom_sec = sec

    def _on_fitness(self, msg: Float64) -> None:
        with self._lock:
            self._fitness_score    = msg.data
            self._fitness_mono_sec = time.monotonic()

    def _on_scan_match(self, msg: PoseWithCovarianceStamped) -> None:
        """Cache the latest accepted LiDAR scan match pose for lidar_pos_error label."""
        with self._lock:
            self._scan_match_x        = msg.pose.pose.position.x
            self._scan_match_y        = msg.pose.pose.position.y
            self._scan_match_mono_sec = time.monotonic()

    # ── Sync callback — one CSV row per call ────────────────────────────────

    def _on_sync(self, gt_msg: Odometry, fgo_msg: Odometry) -> None:
        # ── Timestamps ─────────────────────────────────────────────────────
        stamp = gt_msg.header.stamp
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        # ── Ground truth pose ───────────────────────────────────────────────
        gt_x   = gt_msg.pose.pose.position.x
        gt_y   = gt_msg.pose.pose.position.y
        gt_yaw = _quat_to_yaw(gt_msg.pose.pose.orientation)

        # ── FGO estimated pose ──────────────────────────────────────────────
        fgo_x   = fgo_msg.pose.pose.position.x
        fgo_y   = fgo_msg.pose.pose.position.y
        fgo_yaw = _quat_to_yaw(fgo_msg.pose.pose.orientation)

        # ── Error labels (FGO vs GT) ────────────────────────────────────────
        dx      = fgo_x - gt_x
        dy      = fgo_y - gt_y
        pos_err = math.sqrt(dx * dx + dy * dy)
        yaw_err = abs(_wrap(fgo_yaw - gt_yaw))

        # ── Snapshot cached sensor state ────────────────────────────────────
        with self._lock:
            # Fitness — mark stale if not updated recently
            fitness_score = self._fitness_score
            if not math.isnan(self._fitness_mono_sec):
                fit_age = time.monotonic() - self._fitness_mono_sec
                if fit_age > self._max_fit_age:
                    fitness_score = float("nan")
            else:
                fit_age = float("nan")

            accel_norm  = self._latest_accel_norm
            gyro_z      = self._latest_gyro_z
            accel_buf   = list(self._accel_buf)
            gyro_buf    = list(self._gyro_buf)

            linear_vel  = self._linear_vel
            angular_vel = self._angular_vel
            jerk        = self._jerk

            # Encoder dead-reckoning snapshot
            enc_x_snap   = self._enc_x
            enc_y_snap   = self._enc_y

            # IMU yaw integral snapshot
            imu_yaw_snap = self._imu_yaw_abs

            # LiDAR scan match pose snapshot
            scan_x        = self._scan_match_x
            scan_y        = self._scan_match_y
            scan_mono_sec = self._scan_match_mono_sec

        # ── Derived IMU features (outside lock) ─────────────────────────────
        accel_norm_dev = (abs(accel_norm - self._gravity)
                          if not math.isnan(accel_norm)
                          else float("nan"))
        accel_norm_std = _pop_std(accel_buf)
        gyro_z_std     = _pop_std(gyro_buf)
        slip_metric = (abs(gyro_z - angular_vel)
                       if not (math.isnan(gyro_z) or math.isnan(angular_vel))
                       else float("nan"))

        # ── GT Trust Labels ─────────────────────────────────────────────────

        # 1. LiDAR pos error: distance between accepted scan match pose and GT
        if not math.isnan(scan_mono_sec):
            scan_age = time.monotonic() - scan_mono_sec
            if scan_age <= self._max_scan_age and not math.isnan(scan_x):
                lidar_pos_error = math.sqrt(
                    (scan_x - gt_x) ** 2 + (scan_y - gt_y) ** 2)
            else:
                lidar_pos_error = float("nan")   # stale or no accepted scan
        else:
            lidar_pos_error = float("nan")       # no scan ever received

        # 2 & 3. Encoder drift + IMU yaw error over drift_window
        # Append current state to drift buffer first (time-ordered).
        self._drift_buf.append((t_sec, gt_x, gt_y, gt_yaw, enc_x_snap, enc_y_snap, imu_yaw_snap))

        encoder_drift = float("nan")
        imu_yaw_error = float("nan")

        # Find reference entry: the most-recent entry that is >= drift_window_sec old.
        ref = None
        for entry in self._drift_buf:
            if (t_sec - entry[0]) >= self._drift_window_sec:
                ref = entry      # keep iterating; we want the closest one >= window
            else:
                break            # deque is time-ordered; once within window, stop

        if ref is not None:
            (ref_t, ref_gt_x, ref_gt_y, ref_gt_yaw,
             ref_enc_x, ref_enc_y, ref_imu_yaw) = ref

            # Encoder drift: how much does encoder displacement deviate from GT displacement?
            dgt_x   = gt_x     - ref_gt_x
            dgt_y   = gt_y     - ref_gt_y
            denc_x  = enc_x_snap - ref_enc_x
            denc_y  = enc_y_snap - ref_enc_y
            encoder_drift = math.sqrt(
                (denc_x - dgt_x) ** 2 + (denc_y - dgt_y) ** 2)

            # IMU yaw error: how much does gyro integration deviate from GT yaw change?
            gt_yaw_change  = _wrap(gt_yaw - ref_gt_yaw)
            imu_yaw_change = imu_yaw_snap - ref_imu_yaw   # raw integral (no wrap)
            imu_yaw_error  = abs(_wrap(imu_yaw_change - gt_yaw_change))

        # ── Write CSV row ────────────────────────────────────────────────────
        row = {
            "timestamp_sec":   _fmt(t_sec),
            "fitness_score":   _fmt(fitness_score),
            "fitness_age_sec": _fmt(fit_age, 3),
            "accel_norm_dev":  _fmt(accel_norm_dev),
            "gyro_z":          _fmt(gyro_z),
            "accel_norm_std":  _fmt(accel_norm_std),
            "gyro_z_std":      _fmt(gyro_z_std),
            "linear_vel":      _fmt(linear_vel),
            "angular_vel":     _fmt(angular_vel),
            "jerk":            _fmt(jerk),
            "slip_metric":     _fmt(slip_metric),
            "gt_x":            _fmt(gt_x),
            "gt_y":            _fmt(gt_y),
            "gt_yaw_rad":      _fmt(gt_yaw),
            "fgo_x":           _fmt(fgo_x),
            "fgo_y":           _fmt(fgo_y),
            "fgo_yaw_rad":     _fmt(fgo_yaw),
            "pos_err_m":       _fmt(pos_err),
            "yaw_err_rad":     _fmt(yaw_err),
            "lidar_pos_error": _fmt(lidar_pos_error),
            "encoder_drift":   _fmt(encoder_drift),
            "imu_yaw_error":   _fmt(imu_yaw_error),
        }
        self._csv_writer.writerow(row)
        self._row_count += 1

    # ── Periodic status log ──────────────────────────────────────────────────

    def _log_status(self) -> None:
        with self._lock:
            lin   = self._linear_vel
            fit   = self._fitness_score
            jitr  = self._jerk
            scan_age = (time.monotonic() - self._scan_match_mono_sec
                        if not math.isnan(self._scan_match_mono_sec)
                        else float("nan"))
        self.get_logger().info(
            f"[FeatureExtractor] rows={self._row_count}  "
            f"vel={_fmt(lin, 2)} m/s  fitness={_fmt(fit, 3)}  "
            f"jerk={_fmt(jitr, 3)} m/s²  scan_age={_fmt(scan_age, 1)} s"
        )

    # ── Graceful shutdown ────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._csv_file.flush()
        self._csv_file.close()
        self.get_logger().info(
            f"[FeatureExtractor] Shutdown. CSV closed. "
            f"Total rows written: {self._row_count}"
        )
        super().destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FeatureExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
