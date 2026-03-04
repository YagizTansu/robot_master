#!/usr/bin/env python3
"""
save_amcl_pose.py
─────────────────
Subscribes to /amcl_pose (or remapped /fgo_pose) and saves the latest pose to
~/.ros/last_fgo_pose.json every 5 seconds using an **atomic write** (write to
a temp file then os.replace) so a crash/kill mid-write never leaves a corrupt file.

Usage (FGO):
  ros2 run robot_navigation save_amcl_pose.py
      with remapping:  /amcl_pose:=/fgo_pose
  OR via launch with:  remappings=[('/amcl_pose', '/fgo_pose')]
"""

import json
import os
import tempfile

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# ── Sabit dosya yolu ──────────────────────────────────────────────────────────
POSE_FILE = os.path.expanduser("~/.ros/last_fgo_pose.json")


class PoseSaver(Node):
    def __init__(self):
        super().__init__("fgo_pose_saver")

        # use_sim_time parametresi launch'tan gelir; ROS2 otomatik ayarlar.
        # Buraya özel bir şey yapmamıza gerek yok.

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub_ = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",  # launch remapping ile /fgo_pose'a yönlendirilir
            self._pose_cb,
            qos,
        )

        self._latest_pose: dict | None = None

        # Her 5 saniyede bir atomik kaydet
        self.create_timer(5.0, self._save)

        os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
        self.get_logger().info(f"[PoseSaver] Pose dosyası: {POSE_FILE}")

    # ── Callback ──────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._latest_pose = {
            "x": p.position.x,
            "y": p.position.y,
            "z": p.position.z,
            "qx": p.orientation.x,
            "qy": p.orientation.y,
            "qz": p.orientation.z,
            "qw": p.orientation.w,
        }

    # ── Atomic save ───────────────────────────────────────────────────────────
    def _save(self) -> None:
        if self._latest_pose is None:
            return

        try:
            # Önce aynı dizine geçici dosya yaz, sonra atomik taşı.
            # Bu sayede mid-write crash bozuk JSON bırakmaz.
            dir_ = os.path.dirname(POSE_FILE)
            with tempfile.NamedTemporaryFile(
                mode="w", dir=dir_, suffix=".tmp", delete=False
            ) as tmp:
                json.dump(self._latest_pose, tmp, indent=2)
                tmp_path = tmp.name

            os.replace(tmp_path, POSE_FILE)  # POSIX üzerinde atomic
            self.get_logger().debug(
                f"[PoseSaver] Kaydedildi: x={self._latest_pose['x']:.3f} "
                f"y={self._latest_pose['y']:.3f}"
            )
        except Exception as exc:
            self.get_logger().error(f"[PoseSaver] Kayıt hatası: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
