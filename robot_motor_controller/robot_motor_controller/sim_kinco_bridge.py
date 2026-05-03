#!/usr/bin/env python3
"""
sim_kinco_bridge.py
───────────────────
Simülasyon için kinco_bridge.py'nin Gazebo karşılığı.

Real robot:
  serial encoder reads  →  Ackermann odometry  →  /odom_kinco

Sim robot:
  /joint_states reads   →  Ackermann odometry  →  /odom_kinco

TricycleController zaten /cmd_vel → Gazebo joint komutlarını hallediyor.
Bu node sadece joint state'lerden aynı odometriyi üretir,
navigasyon stack'inin her iki ortamda da /odom_kinco'dan beslenebilmesi için.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# ─── Fiziksel sabitler (kinco_bridge.py ile aynı) ────────────────────────────
WHEEL_RADIUS   = 0.109   # m  — URDF drive_motor_1 cylinder collision radius
WHEEL_BASE     = 1.0957  # m  — URDF direction_motor_joint origin x

# Joint isimleri (boa_new_urdf.xacro ile eşleşmeli)
STEERING_JOINT  = 'direction_motor_joint'
TRACTION_JOINT  = 'drive_motor_joint'


class SimKincoBridge(Node):

    def __init__(self):
        super().__init__('sim_kinco_bridge')

        # Odometry durumu
        self._pos_x     = 0.0
        self._pos_y     = 0.0
        self._pos_theta = 0.0
        self._last_time = self.get_clock().now()
        self._lock      = threading.Lock()

        # Son joint değerleri
        self._steering_angle = 0.0   # rad — direction_motor_joint position
        self._wheel_vel      = 0.0   # rad/s — drive_motor_joint velocity

        self._odom_pub = self.create_publisher(Odometry, '/odom_kinco', 10)

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Odometri 50 Hz timer
        self.create_timer(1.0 / 50.0, self._odom_timer_cb)

        self.get_logger().info(
            f'sim_kinco_bridge başlatıldı — '
            f'wheelbase={WHEEL_BASE} m, wheel_radius={WHEEL_RADIUS} m')

    # ─── Joint state callback ─────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name == STEERING_JOINT:
                    self._steering_angle = msg.position[i]  # rad
                elif name == TRACTION_JOINT:
                    self._wheel_vel = msg.velocity[i]       # rad/s

    # ─── 50 Hz odometry timer ─────────────────────────────────────────────────
    def _odom_timer_cb(self):
        current_time = self.get_clock().now()

        with self._lock:
            steering_angle = self._steering_angle
            wheel_vel      = self._wheel_vel

        dt = (current_time - self._last_time).nanoseconds * 1e-9
        self._last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        # ── Ackermann kinematiği ─────────────────────────────────────────────────────
        #   v_robot = wheel_vel [rad/s] * wheel_radius [m] * cos(delta)
        #   omega   = v_robot * tan(delta) / wheelbase
        wheel_speed      = wheel_vel * WHEEL_RADIUS
        linear_velocity  = wheel_speed * math.cos(steering_angle)
        angular_velocity = math.tan(steering_angle) * linear_velocity / WHEEL_BASE

        # ── Pose integrasyon ─────────────────────────────────────────────────
        self._pos_x     += linear_velocity * math.cos(self._pos_theta) * dt
        self._pos_y     += linear_velocity * math.sin(self._pos_theta) * dt
        self._pos_theta += angular_velocity * dt

        # ── Quaternion (yaw) ──────────────────────────────────────────────────
        qz = math.sin(self._pos_theta / 2.0)
        qw = math.cos(self._pos_theta / 2.0)

        stamp = current_time.to_msg()

        # ── Odometry mesajı ───────────────────────────────────────────────────
        # TF yayını EKF local node'a bırakıldı (çift yayın kayma yapardı)
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = self._pos_x
        odom.pose.pose.position.y    = self._pos_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Pose covariance (6x6): x, y, z, roll, pitch, yaw
        # 2D robot: z/roll/pitch kullanılmaz → 1e6 ile devre dışı
        odom.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
        ]

        odom.twist.twist.linear.x  = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Twist covariance (6x6): vx, vy, vz, vroll, vpitch, vyaw
        # Ackermann/forklift: yanal hız yok → vy 1e6
        odom.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e6,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05,
        ]
        
        self._odom_pub.publish(odom)



def main():
    rclpy.init()
    node = SimKincoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
