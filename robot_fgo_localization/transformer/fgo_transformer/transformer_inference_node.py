#!/usr/bin/env python3
"""
Transformer Inference Node
==========================
Loads the trained TrustTransformer checkpoint, subscribes to the same sensor
topics as feature_extractor_node, and publishes trust-weight predictions to
/transformer/trust_scores at ~10 Hz.

TrustWeightBridge consumes this topic when manual_mode = false.

Published Topics
----------------
/transformer/trust_scores   std_msgs/Float64MultiArray
    data = [w_encoder, w_imu, w_lidar, w_gps]  all ∈ (EPSILON, 1.0]

Usage
-----
    ros2 launch fgo_transformer transformer_inference.launch.py

Then switch FGO to Transformer mode:
    ros2 service call /fgo/set_trust_weights std_srvs/srv/SetBool "{data: false}"
"""

import collections
import json
import math
import os
import threading
import time

import numpy as np

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Float64MultiArray

# ─────────────────────────────────────────────────────────────────────────────
# Lazy-load torch so the node starts even without GPU
# ─────────────────────────────────────────────────────────────────────────────
try:
    import torch
    _TORCH_AVAILABLE = True
except ImportError:
    _TORCH_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Feature column order — must match training/config.py FEATURE_COLS exactly
# ─────────────────────────────────────────────────────────────────────────────
_FEATURE_COLS = [
    "fitness_score",
    "fitness_age_sec",
    "accel_norm_dev",
    "gyro_z",
    "accel_norm_std",
    "gyro_z_std",
    "linear_vel",
    "angular_vel",
    "jerk",
    "slip_metric",
]
_N_FEATURES = len(_FEATURE_COLS)
_TRUST_EPSILON = 1e-6


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _pop_std(buf) -> float:
    n = len(buf)
    if n < 2:
        return float("nan")
    mean = sum(buf) / n
    return math.sqrt(sum((x - mean) ** 2 for x in buf) / n)


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class TransformerInferenceNode(Node):

    def __init__(self) -> None:
        super().__init__("transformer_inference")

        # ── Parameters ─────────────────────────────────────────────────────
        _pkg_share = get_package_share_directory("fgo_transformer")
        _default_ckpt = os.path.join(_pkg_share, "data", "checkpoints", "best.pt")

        self.declare_parameter("checkpoint_path",     _default_ckpt)
        self.declare_parameter("publish_rate_hz",     10.0)
        self.declare_parameter("seq_len",             20)
        self.declare_parameter("imu_buffer_size",     50)
        self.declare_parameter("gravity",             9.81)
        self.declare_parameter("max_fitness_age_sec", 2.0)
        self.declare_parameter("nan_fill",            0.0)

        ckpt_path         = str(self.get_parameter("checkpoint_path").value)
        publish_rate_hz   = float(self.get_parameter("publish_rate_hz").value)
        self._seq_len     = int(self.get_parameter("seq_len").value)
        imu_buf_size      = int(self.get_parameter("imu_buffer_size").value)
        self._gravity     = float(self.get_parameter("gravity").value)
        self._max_fit_age = float(self.get_parameter("max_fitness_age_sec").value)
        self._nan_fill    = float(self.get_parameter("nan_fill").value)

        # ── Load model ──────────────────────────────────────────────────────
        if not _TORCH_AVAILABLE:
            self.get_logger().fatal("[TransformerInference] PyTorch not installed!")
            raise RuntimeError("PyTorch not available")

        self._model, self._norm_stats = self._load_checkpoint(ckpt_path)
        self._device = next(self._model.parameters()).device

        # ── Sliding feature window  [seq_len, n_features] ───────────────────
        self._lock = threading.Lock()
        # Rolling window of raw (un-normalised) feature rows
        self._window: collections.deque = collections.deque(
            maxlen=self._seq_len)

        # IMU rolling buffers
        self._accel_buf: collections.deque = collections.deque(maxlen=imu_buf_size)
        self._gyro_buf:  collections.deque = collections.deque(maxlen=imu_buf_size)
        self._latest_accel_norm: float = float("nan")
        self._latest_gyro_z:    float = float("nan")

        # Wheel odometry
        self._linear_vel:    float = float("nan")
        self._angular_vel:   float = float("nan")
        self._jerk:          float = float("nan")
        self._prev_lin_vel:  float = float("nan")
        self._prev_odom_sec: float = float("nan")

        # NDT fitness
        self._fitness_score:    float = float("nan")
        self._fitness_mono_sec: float = float("nan")

        # Last published weights (used as fallback if window not full yet)
        self._last_trust = [1.0, 1.0, 1.0, 0.0]   # [enc, imu, lidar, gps]

        # ── QoS ─────────────────────────────────────────────────────────────
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

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Imu,      "/imu",                      self._on_imu,     sensor_qos)
        self.create_subscription(Odometry, "/odometry",                 self._on_odom,    reliable_qos)
        self.create_subscription(Float64,  "/scan_match/fitness_score", self._on_fitness, reliable_qos)

        # ── Publisher ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            Float64MultiArray, "/transformer/trust_scores", reliable_qos)

        # ── Inference timer ──────────────────────────────────────────────────
        period_ms = int(1000.0 / publish_rate_hz)
        self.create_timer(period_ms / 1000.0, self._inference_and_publish)

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  TransformerInferenceNode started\n"
            f"  Checkpoint : {ckpt_path}\n"
            f"  Device     : {self._device}\n"
            f"  seq_len    : {self._seq_len}  publish_rate: {publish_rate_hz} Hz\n"
            f"{'='*60}"
        )

    # ── Checkpoint loader ────────────────────────────────────────────────────

    def _load_checkpoint(self, path: str):
        import sys
        # Add training package to path so ModelConfig can be unpickled
        training_dir = os.path.join(
            os.path.dirname(__file__), "..", "training")
        if training_dir not in sys.path:
            sys.path.insert(0, os.path.abspath(training_dir))

        # Import here (after path fix) to avoid top-level import at module load
        from training.model import TrustTransformer    # noqa: PLC0415

        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            self.get_logger().fatal(f"[TransformerInference] Checkpoint not found: {path}")
            raise FileNotFoundError(path)

        ckpt = torch.load(path, map_location="cpu", weights_only=False)
        model_cfg  = ckpt["model_cfg"]
        norm_stats = ckpt["norm_stats"]

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model = TrustTransformer(model_cfg).to(device)
        model.load_state_dict(ckpt["model_state"])
        model.eval()

        self.get_logger().info(
            f"[TransformerInference] Loaded checkpoint (epoch={ckpt['epoch']}, "
            f"val_loss={ckpt['val_loss']:.5f})"
        )
        return model, norm_stats

    # ── Sensor callbacks (same logic as feature_extractor_node) ──────────────

    def _on_imu(self, msg: Imu) -> None:
        ax, ay, az = (msg.linear_acceleration.x,
                      msg.linear_acceleration.y,
                      msg.linear_acceleration.z)
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        gz   = msg.angular_velocity.z
        with self._lock:
            self._accel_buf.append(norm)
            self._gyro_buf.append(gz)
            self._latest_accel_norm = norm
            self._latest_gyro_z     = gz

    def _on_odom(self, msg: Odometry) -> None:
        vx  = msg.twist.twist.linear.x
        wz  = msg.twist.twist.angular.z
        sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if (not math.isnan(self._prev_lin_vel)
                    and not math.isnan(self._prev_odom_sec)):
                dt = sec - self._prev_odom_sec
                self._jerk = abs(vx - self._prev_lin_vel) / dt if dt > 1e-6 else 0.0
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

    # ── Build one feature row from cached state ───────────────────────────────

    def _build_feature_row(self) -> list:
        """Returns a list of _N_FEATURES floats (NaN if sensor not ready)."""
        with self._lock:
            fitness = self._fitness_score
            if not math.isnan(self._fitness_mono_sec):
                fit_age = time.monotonic() - self._fitness_mono_sec
                if fit_age > self._max_fit_age:
                    fitness = float("nan")
            else:
                fit_age = float("nan")

            accel_norm  = self._latest_accel_norm
            gyro_z      = self._latest_gyro_z
            accel_buf   = list(self._accel_buf)
            gyro_buf    = list(self._gyro_buf)
            lin         = self._linear_vel
            ang         = self._angular_vel
            jerk        = self._jerk

        accel_dev = (abs(accel_norm - self._gravity)
                     if not math.isnan(accel_norm) else float("nan"))
        accel_std = _pop_std(accel_buf)
        gyro_std  = _pop_std(gyro_buf)
        slip      = (abs(gyro_z - ang)
                     if not (math.isnan(gyro_z) or math.isnan(ang))
                     else float("nan"))

        return [fitness, fit_age, accel_dev, gyro_z, accel_std,
                gyro_std, lin, ang, jerk, slip]

    # ── Normalise a single row ────────────────────────────────────────────────

    def _normalise(self, row: list) -> np.ndarray:
        out = np.empty(_N_FEATURES, dtype=np.float32)
        for i, col in enumerate(_FEATURE_COLS):
            v = row[i]
            v = self._nan_fill if (isinstance(v, float) and math.isnan(v)) else v
            stats = self._norm_stats[col]
            out[i] = (v - stats["mean"]) / stats["std"]
        return out

    # ── Inference timer callback ─────────────────────────────────────────────

    def _inference_and_publish(self) -> None:
        # 1. Build and append one feature row to the sliding window
        row = self._build_feature_row()
        norm_row = self._normalise(row)

        with self._lock:
            self._window.append(norm_row)
            window_snapshot = list(self._window)

        # 2. Run inference only when the window is full
        if len(window_snapshot) < self._seq_len:
            # Publish safe fallback weights until window fills up
            self._publish(self._last_trust)
            return

        # 3. Stack into tensor [seq_len, n_features] → batch [1, seq, feat]
        window_arr = np.stack(window_snapshot, axis=0)          # [seq, feat]
        x = torch.from_numpy(window_arr).unsqueeze(0).to(self._device)  # [1,seq,feat]

        with torch.no_grad():
            trust_t, _, _ = self._model(x)                       # [1, 4]; yaw discarded
        trust = trust_t.squeeze(0).cpu().tolist()                # [4]

        # GPS is disabled (no GPSFactor in FGO) — always force to 0.
        # Other weights are clamped to [EPSILON, 1.0] for numerical safety.
        _EPS = 1e-6
        trust[0] = max(_EPS, min(1.0, trust[0]))  # encoder
        trust[1] = max(_EPS, min(1.0, trust[1]))  # imu
        trust[2] = max(_EPS, min(1.0, trust[2]))  # lidar
        trust[3] = 0.0                             # gps — disabled

        self._last_trust = trust
        self._publish(trust)

        self.get_logger().debug(
            f"[TransformerInference] "
            f"enc={trust[0]:.3f}  imu={trust[1]:.3f}  "
            f"lidar={trust[2]:.3f}  gps={trust[3]:.3f}"
        )

    def _publish(self, trust: list) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in trust]
        self._pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TransformerInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
