#!/usr/bin/env python3
"""
test_trust_weights.py — Integration test for the FGO trust-weight scaling layer.

Self-contained: does NOT require an external benchmark node.

What it measures per scenario:
  - relay_ok      : whether /fgo/trust_weights echoes the correct values
  - relay_latency : round-trip time from publish → receive (ms)
  - pose_samples  : number of /fgo/odometry messages received
  - pose_disp_m   : total XY displacement of the robot during the scenario (m)
  - pose_std_m    : standard deviation of per-step XY displacement (m)
                    (higher std under degraded weights may indicate noisier estimates)

Results are saved to ~/ros2_ws/results/trust_weight_test_results.csv.

Usage (after building and sourcing the workspace):
    ros2 run factor_graph_optimization test_trust_weights.py

Requires:
  - trust_weight_bridge node running
  - fgo_node running and publishing /fgo/odometry
"""

import csv
import math
import os
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


# ─────────────────────────────────────────────────────────────────────────────
# Scenario definitions
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Scenario:
    name: str
    description: str
    w_encoder: float = 1.0
    w_imu:     float = 1.0
    w_lidar:   float = 1.0
    w_gps:     float = 1.0
    duration_s: float = 10.0

    # Collected during run
    relay_latencies_ms: List[float] = field(default_factory=list)
    relay_mismatches:   int = 0
    pose_steps:         List[float] = field(default_factory=list)  # per-step XY displacement
    relay_ok:           bool = False

    def weights(self) -> List[float]:
        return [self.w_encoder, self.w_imu, self.w_lidar, self.w_gps]


SCENARIOS: List[Scenario] = [
    Scenario(
        name="A_baseline",
        description="Baseline — all weights = 1.0 (no scaling)",
        w_encoder=1.0, w_imu=1.0, w_lidar=1.0, w_gps=1.0,
    ),
    Scenario(
        name="B_encoder_degraded",
        description="Encoder degradation — simulates wheel slip (w_encoder=0.1)",
        w_encoder=0.1, w_imu=1.0, w_lidar=1.0, w_gps=1.0,
    ),
    Scenario(
        name="C_lidar_degraded",
        description="LiDAR degradation — simulates featureless corridor (w_lidar=0.1)",
        w_encoder=1.0, w_imu=1.0, w_lidar=0.1, w_gps=1.0,
    ),
    Scenario(
        name="D_gps_degraded",
        description="GPS degradation — simulates urban canyon (w_gps=0.1)",
        w_encoder=1.0, w_imu=1.0, w_lidar=1.0, w_gps=0.1,
    ),
    Scenario(
        name="E_multi_degraded",
        description="Multi-sensor degradation — IMU+LiDAR carry the system",
        w_encoder=0.1, w_imu=0.9, w_lidar=0.9, w_gps=0.1,
    ),
]

_TOL = 1e-4  # tolerance for weight value comparison


# ─────────────────────────────────────────────────────────────────────────────
# Test node
# ─────────────────────────────────────────────────────────────────────────────

class TrustWeightTestNode(Node):

    RESULTS_CSV = os.path.expanduser("~/ros2_ws/results/trust_weight_test_results.csv")

    def __init__(self) -> None:
        super().__init__("trust_weight_test_node")

        pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)

        # Publisher: send weights to the bridge
        self._pub_scores = self.create_publisher(
            Float64MultiArray, "/transformer/trust_scores", pub_qos)

        # Subscriber: verify bridge relayed them correctly
        self._last_received: Optional[List[float]] = None
        self._last_recv_time: Optional[float] = None
        self.create_subscription(
            Float64MultiArray, "/fgo/trust_weights",
            self._on_trust_weights, pub_qos)

        # Subscriber: /fgo/odometry — pose displacement proxy
        self._last_pose: Optional[Tuple[float, float]] = None
        self.create_subscription(
            Odometry, "/fgo/odometry",
            self._on_odometry, 10)

        self._current_scenario: Optional[Scenario] = None
        self._last_publish_time: Optional[float] = None

        self._mode_client = self.create_client(SetBool, "/fgo/set_trust_weights")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_trust_weights(self, msg: Float64MultiArray) -> None:
        self._last_received  = list(msg.data)
        self._last_recv_time = time.monotonic()

        sc = self._current_scenario
        if sc is None or self._last_publish_time is None:
            return

        # Relay latency
        latency_ms = (self._last_recv_time - self._last_publish_time) * 1000.0
        sc.relay_latencies_ms.append(latency_ms)

        # Value correctness check
        expected = sc.weights()
        if len(msg.data) >= 4:
            mismatch = any(
                abs(msg.data[i] - expected[i]) > _TOL for i in range(4))
            if mismatch:
                sc.relay_mismatches += 1
            else:
                sc.relay_ok = True

    def _on_odometry(self, msg: Odometry) -> None:
        sc = self._current_scenario
        if sc is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_pose is not None:
            dx = x - self._last_pose[0]
            dy = y - self._last_pose[1]
            sc.pose_steps.append(math.hypot(dx, dy))
        self._last_pose = (x, y)

    # ── Helpers ────────────────────────────────────────────────────────────

    def _publish_weights(self, weights: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = weights
        self._pub_scores.publish(msg)
        self._last_publish_time = time.monotonic()

    def _switch_to_transformer_mode(self) -> bool:
        if not self._mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "[TestNode] /fgo/set_trust_weights service not available. "
                "Continuing — ensure trust_weight_bridge is running with manual_mode=false.")
            return False
        req = SetBool.Request()
        req.data = False  # false = Transformer mode
        future = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and future.result().success:
            self.get_logger().info("[TestNode] Switched to Transformer mode.")
            return True
        self.get_logger().warn("[TestNode] Failed to switch to Transformer mode.")
        return False

    def _run_scenario(self, scenario: Scenario, publish_hz: float = 10.0) -> None:
        self._current_scenario = scenario
        self._last_pose = None
        interval = 1.0 / publish_hz
        end_time = time.monotonic() + scenario.duration_s

        self.get_logger().info(
            f"[TestNode] ▶ {scenario.name}: "
            f"encoder={scenario.w_encoder}  imu={scenario.w_imu}  "
            f"lidar={scenario.w_lidar}  gps={scenario.w_gps}  "
            f"({scenario.duration_s}s)")

        while time.monotonic() < end_time:
            self._publish_weights(scenario.weights())
            rclpy.spin_once(self, timeout_sec=interval)

        self._current_scenario = None
        n_relay = len(scenario.relay_latencies_ms)
        n_pose  = len(scenario.pose_steps)
        self.get_logger().info(
            f"[TestNode] ✓ {scenario.name}: "
            f"relay_ok={scenario.relay_ok}  "
            f"relay_msgs={n_relay}  mismatches={scenario.relay_mismatches}  "
            f"odom_steps={n_pose}")

    # ── Main sequence ──────────────────────────────────────────────────────

    def run(self) -> None:
        self.get_logger().info("[TestNode] Starting trust-weight integration test.")
        self._switch_to_transformer_mode()

        # Warm-up
        self.get_logger().info("[TestNode] Warm-up (2 s)...")
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        for scenario in SCENARIOS:
            self._run_scenario(scenario)

        self._save_results()
        self.get_logger().info(f"[TestNode] Results → {self.RESULTS_CSV}")

    def _save_results(self) -> None:
        import statistics

        rows = []
        for sc in SCENARIOS:
            if sc.relay_latencies_ms:
                lat_mean = statistics.mean(sc.relay_latencies_ms)
                lat_max  = max(sc.relay_latencies_ms)
            else:
                lat_mean = lat_max = float("nan")

            if sc.pose_steps:
                pose_disp = sum(sc.pose_steps)
                pose_std  = statistics.stdev(sc.pose_steps) if len(sc.pose_steps) > 1 else 0.0
            else:
                pose_disp = pose_std = float("nan")

            rows.append({
                "scenario":          sc.name,
                "description":       sc.description,
                "w_encoder":         sc.w_encoder,
                "w_imu":             sc.w_imu,
                "w_lidar":           sc.w_lidar,
                "w_gps":             sc.w_gps,
                "relay_ok":          sc.relay_ok,
                "relay_msgs":        len(sc.relay_latencies_ms),
                "relay_mismatches":  sc.relay_mismatches,
                "lat_mean_ms":       round(lat_mean, 3),
                "lat_max_ms":        round(lat_max, 3),
                "odom_steps":        len(sc.pose_steps),
                "pose_disp_m":       round(pose_disp, 4) if not math.isnan(pose_disp) else "nan",
                "pose_step_std_m":   round(pose_std,  5) if not math.isnan(pose_std)  else "nan",
            })

        fieldnames = list(rows[0].keys())
        os.makedirs(os.path.dirname(self.RESULTS_CSV), exist_ok=True)
        with open(self.RESULTS_CSV, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        # Terminal summary
        hdr = f"{'Scenario':<25} {'relay_ok':>8} {'lat_mean_ms':>12} {'pose_disp_m':>12} {'step_std_m':>11}"
        sep = "─" * len(hdr)
        self.get_logger().info(sep)
        self.get_logger().info(hdr)
        self.get_logger().info(sep)
        for r in rows:
            self.get_logger().info(
                f"{r['scenario']:<25} {str(r['relay_ok']):>8} "
                f"{str(r['lat_mean_ms']):>12} {str(r['pose_disp_m']):>12} "
                f"{str(r['pose_step_std_m']):>11}")
        self.get_logger().info(sep)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TrustWeightTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("[TestNode] Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
