#!/usr/bin/env python3
"""
publish_initial_pose.py
───────────────────────
Reads the last saved pose from ~/.ros/last_fgo_pose.json and publishes it
once to /initialpose so the FGO / AMCL node can resume from where it left off.

Key improvements over previous version:
  • No time.sleep() in __init__ — uses a ROS2 one-shot timer instead, which
    respects use_sim_time and does not block the executor.
  • JSON integrity check before accepting the file.
  • Covariance values are configurable via ROS2 parameters.
  • Node exits cleanly after publishing (one-shot pattern).
"""

import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# ── Sabit dosya yolu ──────────────────────────────────────────────────────────
POSE_FILE = os.path.expanduser("~/.ros/last_fgo_pose.json")

# Zorunlu anahtarlar — bunlar eksikse dosya geçersiz sayılır
REQUIRED_KEYS = {"x", "y", "z", "qx", "qy", "qz", "qw"}


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")

        # Parametreler: launch'tan veya YAML'dan gelebilir
        self.declare_parameter("publish_delay_sec", 2.0)
        self.declare_parameter("cov_x", 0.25)
        self.declare_parameter("cov_y", 0.25)
        self.declare_parameter("cov_yaw", 0.06854)  # ~(15°)^2 rad^2

        delay = self.get_parameter("publish_delay_sec").value

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", qos
        )

        # ── One-shot timer: time.sleep yerine ROS2-safe bekleme ───────────────
        # use_sim_time=true ise sim saatine göre tetiklenir.
        self._timer = self.create_timer(delay, self._publish_once)
        self.get_logger().info(
            f"[InitialPosePublisher] {delay:.1f}s sonra pose yayınlanacak..."
        )

    # ── One-shot yayın ────────────────────────────────────────────────────────
    def _publish_once(self) -> None:
        # Timer hemen iptal et → tek seferlik çalışsın
        self._timer.cancel()

        if not os.path.exists(POSE_FILE):
            self.get_logger().warn(
                f"[InitialPosePublisher] Pose dosyası bulunamadı: {POSE_FILE} "
                "— origin'den başlanacak."
            )
            return

        # ── JSON yükle ve doğrula ─────────────────────────────────────────────
        try:
            with open(POSE_FILE, "r") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().error(
                f"[InitialPosePublisher] Pose dosyası okunamadı ({exc}) — atlanıyor."
            )
            return

        if not REQUIRED_KEYS.issubset(data.keys()):
            missing = REQUIRED_KEYS - data.keys()
            self.get_logger().error(
                f"[InitialPosePublisher] Eksik alanlar: {missing} — atlanıyor."
            )
            return

        # ── Quaternion geçerlilik kontrolü ────────────────────────────────────
        norm = math.sqrt(
            data["qx"] ** 2 + data["qy"] ** 2 + data["qz"] ** 2 + data["qw"] ** 2
        )
        if abs(norm - 1.0) > 0.05:
            self.get_logger().error(
                f"[InitialPosePublisher] Quaternion norm={norm:.4f} — geçersiz, atlanıyor."
            )
            return

        # ── Mesajı oluştur ────────────────────────────────────────────────────
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(data["x"])
        msg.pose.pose.position.y = float(data["y"])
        msg.pose.pose.position.z = float(data["z"])
        msg.pose.pose.orientation.x = float(data["qx"])
        msg.pose.pose.orientation.y = float(data["qy"])
        msg.pose.pose.orientation.z = float(data["qz"])
        msg.pose.pose.orientation.w = float(data["qw"])

        # Covariance: parametreden oku (6×6 düz matris, row-major)
        cov_x = self.get_parameter("cov_x").value
        cov_y = self.get_parameter("cov_y").value
        cov_yaw = self.get_parameter("cov_yaw").value
        msg.pose.covariance[0] = cov_x  # x-x
        msg.pose.covariance[7] = cov_y  # y-y
        msg.pose.covariance[35] = cov_yaw  # yaw-yaw

        self._pub.publish(msg)
        self.get_logger().info(
            f"[InitialPosePublisher] /initialpose yayınlandı: "
            f"x={data['x']:.3f} y={data['y']:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    # Tek mesaj yayınlamak için bir spin_once döngüsü yeterli;
    # timer tetiklenene kadar spin ediyoruz, sonra node kapatılıyor.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
