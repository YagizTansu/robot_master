#!/usr/bin/env python3
"""
sim_slamware_bridge.py
======================
Simülasyon için slamware_bridge.py'nin Gazebo karşılığı.

Gerçek robot:
  Slamware Aurora (onboard SLAM)
  → /slamware_ros_sdk_server_node/odom  (Odometry, slamware kendi koordinatında)
  → slamware_bridge.py  (robot merkezi offset düzeltmesi: -0.85 m, frame_id="odom")
  → /slamware_odom  (PoseWithCovarianceStamped)

Simülasyon:
  /ground_truth/odom  (Gazebo gerçek konumu, map frame)
  → sim_slamware_bridge.py  (Gaussian gürültü + frame_id="odom")
  → /slamware_odom  (PoseWithCovarianceStamped)

Neden aynı yaklaşım çalışır:
  - Slamware, lokalizasyon modunda robotun haritadaki mutlak konumunu yayınlar.
    slamware_bridge.py bunu "odom" frame'i olarak etiketler çünkü Slamware'in
    iç koordinat sistemi odom frame'i ile hizalanmıştır.
  - Simülasyonda ground_truth zaten harita koordinatındadır; statik identity
    TF (map→odom) ile odom≈map olduğundan aynı mantık geçerlidir.

EKF (ekf_slamware.yaml) her iki ortamda DEĞİŞMEDEN çalışır:
  odom0: /odom_kinco     [vx, vyaw — tekerlek odometrisi]
  pose0: /slamware_odom  [x, y, yaw — mutlak konum]
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SimSlamwareBridge(Node):

    def __init__(self):
        super().__init__('sim_slamware_bridge')

        # ── Gürültü parametreleri ─────────────────────────────────────────────
        # Slamware Aurora tipik lokalizasyon doğruluğu:
        #   pozisyon: ~2-5 cm RMS,  yaw: ~0.3-0.5 derece RMS
        self.declare_parameter('noise_x_std',   0.02)   # m
        self.declare_parameter('noise_y_std',   0.02)   # m
        self.declare_parameter('noise_yaw_std', 0.005)  # rad (~0.3 derece)
        self.declare_parameter('use_noise',     True)

        self._nx    = self.get_parameter('noise_x_std').get_parameter_value().double_value
        self._ny    = self.get_parameter('noise_y_std').get_parameter_value().double_value
        self._nyaw  = self.get_parameter('noise_yaw_std').get_parameter_value().double_value
        self._noisy = self.get_parameter('use_noise').get_parameter_value().bool_value

        # ── Publisher / Subscriber ────────────────────────────────────────────
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/slamware_odom', 10)

        self.create_subscription(
            Odometry, '/ground_truth/odom', self._gt_cb, 10)

        self.get_logger().info(
            f'sim_slamware_bridge başlatıldı — '
            f'gürültü: {"açık" if self._noisy else "kapalı"} '
            f'(σx={self._nx:.3f} m, σy={self._ny:.3f} m, '
            f'σyaw={math.degrees(self._nyaw):.2f}°)')

    # ── Callback ──────────────────────────────────────────────────────────────
    def _gt_cb(self, msg: Odometry):
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        # Slamware bridge'le aynı davranış: frame_id = "odom"
        # (map→odom static identity TF ile map ≡ odom)
        out.header.frame_id = 'odom'

        out.pose.pose = msg.pose.pose

        if self._noisy:
            # Pozisyon gürültüsü
            out.pose.pose.position.x += float(np.random.normal(0.0, self._nx))
            out.pose.pose.position.y += float(np.random.normal(0.0, self._ny))

            # Yaw gürültüsü — mevcut yaw'ı çıkar, gürültü ekle, quaternion yeniden oluştur
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw  = math.atan2(siny, cosy) + float(np.random.normal(0.0, self._nyaw))

            out.pose.pose.orientation.x = 0.0
            out.pose.pose.orientation.y = 0.0
            out.pose.pose.orientation.z = math.sin(yaw / 2.0)
            out.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # ── Kovaryans (6×6, satır-büyük: x, y, z, roll, pitch, yaw) ─────────
        # Slamware pose0_pose_rejection_threshold=5.0 ile uyumlu değerler.
        # z/roll/pitch kullanılmadığından 1e6 ile devre dışı.
        sx2   = self._nx   ** 2
        sy2   = self._ny   ** 2
        syaw2 = self._nyaw ** 2
        out.pose.covariance = [
            sx2,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  sy2,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  syaw2,
        ]

        self._pub.publish(out)


def main():
    rclpy.init()
    node = SimSlamwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
