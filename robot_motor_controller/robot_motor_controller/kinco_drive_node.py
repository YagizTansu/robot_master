#!/usr/bin/env python3
"""
kinco_drive_node.py
───────────────────
Simülasyonda kinco_bridge.py ile AYNI kinematik mantığı uygular.

Real robot (kinco_bridge.py):
  /cmd_vel → compute_commands() → serial RS-232 → Kinco motor

Sim (kinco_drive_node.py):
  /cmd_vel → compute_commands() → /steering_controller/commands  (position, rad)
                                → /traction_controller/commands  (velocity, rad/s)

compute_commands() fonksiyonu birebir kinco_bridge.py ile aynıdır.
Böylece real ve sim tamamen aynı sürüş davranışını sergiler.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# ─── Fiziksel sabitler (kinco_bridge.py ile aynı) ────────────────────────────
WHEEL_DIAMETER = 0.218   # m  (2 * 0.109)
WHEEL_BASE     = 1.0957  # m
TRACTION_GEAR  = 16.4    # motor devri / tekerlek devri


def compute_commands(vx: float, wz: float):
    """
    kinco_bridge.py ile birebir aynı fonksiyon.

    (vx m/s, wz rad/s) -> (steering_angle_rad, motor_rpm, rotate_in_place)

    steering_angle_rad : direction_motor_joint position komutu (rad)
    motor_rpm          : traction motor RPM (→ wheel rad/s'ye çevrilecek)
    rotate_in_place    : True ise direksiyon hazır olana kadar bekle
    """
    if abs(vx) > 1e-3:
        steering_angle  = math.atan2(wz * WHEEL_BASE, vx)
        traction_speed  = vx / math.cos(steering_angle)
        rotate_in_place = False

    elif abs(wz) > 1e-3:
        steering_angle  = math.atan2(wz * WHEEL_BASE, 1e-6)
        traction_speed  = max(-0.3, min(0.3, abs(wz) * WHEEL_BASE))
        rotate_in_place = True

    else:
        return 0.0, 0.0, False

    # +-90 derece clamp (geri hareket için yön çevir)
    if steering_angle > math.pi / 2:
        steering_angle -= math.pi
        traction_speed *= -1
    elif steering_angle < -math.pi / 2:
        steering_angle += math.pi
        traction_speed *= -1

    wheel_rpm = (60.0 * traction_speed) / (math.pi * WHEEL_DIAMETER)
    motor_rpm = wheel_rpm * TRACTION_GEAR

    return steering_angle, motor_rpm, rotate_in_place


class KincoDriveNode(Node):

    def __init__(self):
        super().__init__('kinco_drive_node')

        # Steering: position komutunu direction_motor_joint'e gönder (rad)
        self._steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)

        # Traction: velocity komutunu drive_motor_joint'e gönder (rad/s)
        self._traction_pub = self.create_publisher(
            Float64MultiArray, '/traction_controller/commands', 10)

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self.get_logger().info(
            'kinco_drive_node başlatıldı — '
            f'wheelbase={WHEEL_BASE} m, wheel_diameter={WHEEL_DIAMETER} m, '
            f'traction_gear={TRACTION_GEAR}')

    def _cmd_vel_cb(self, msg: Twist):
        steering_angle, motor_rpm, _ = compute_commands(msg.linear.x, msg.angular.z)

        # motor_rpm → tekerlek açısal hızı (rad/s)
        # wheel_rpm = motor_rpm / TRACTION_GEAR
        # wheel_rad_s = wheel_rpm * 2π / 60
        wheel_rad_s = (motor_rpm / TRACTION_GEAR) * (2.0 * math.pi / 60.0)

        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_angle]
        self._steering_pub.publish(steering_msg)

        traction_msg = Float64MultiArray()
        traction_msg.data = [wheel_rad_s]
        self._traction_pub.publish(traction_msg)


def main():
    rclpy.init()
    node = KincoDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Dururken motor ve direksiyon sıfırla
        stop_steering = Float64MultiArray()
        stop_steering.data = [0.0]
        stop_traction = Float64MultiArray()
        stop_traction.data = [0.0]
        node._steering_pub.publish(stop_steering)
        node._traction_pub.publish(stop_traction)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
