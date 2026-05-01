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
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ─── Fiziksel sabitler (kinco_bridge.py ile aynı) ────────────────────────────
WHEEL_DIAMETER  = 0.218    # m  (2 * 0.109)
WHEEL_BASE      = 1.0957   # m
TRACTION_GEAR   = 16.4     # motor devri / tekerlek devri

# Steering P controller — pozisyon hatası → velocity komutu (rad/s)
STEERING_KP      = 8.0    # rad/s per rad error
STEERING_MAX_VEL = 3.0    # rad/s — max steering velocity
STEERING_JOINT   = 'direction_motor_joint'

# Slew rate limiter — steering hedefi max bu hızda değişir (rad/s)
# Sallanmayı önler, geçişleri yumuşatır
STEERING_SLEW_RATE = 1.5  # rad/s  (0→90° yaklaşık 1 saniye)


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
        # Tam yerinde dönüş: ±90° + doğru traction hızı
        # v_wheel = ω * L / sin(±90°) = ω * L
        steering_angle  = math.copysign(math.pi / 2, wz)
        traction_speed  = abs(wz) * WHEEL_BASE   # m/s — cap yok
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

        # Steering: velocity komutu (P controller ile position tracking)
        self._steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)

        # Traction: velocity komutu (rad/s)
        self._traction_pub = self.create_publisher(
            Float64MultiArray, '/traction_controller/commands', 10)

        # Mevcut steering açısını oku
        self._current_steering   = 0.0
        self._target_steering    = 0.0
        self._slewed_steering    = 0.0   # slew rate limiter çıkışı
        self._target_wheel_radps = 0.0
        self._rotate_in_place    = False

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # 50 Hz steering P controller loop
        self.create_timer(1.0 / 50.0, self._steering_control_loop)

        self.get_logger().info(
            'kinco_drive_node başlatıldı — '
            f'wheelbase={WHEEL_BASE} m, Kp={STEERING_KP}')

    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name == STEERING_JOINT:
                self._current_steering = msg.position[i]
                break

    def _cmd_vel_cb(self, msg: Twist):
        steering_angle, motor_rpm, rotate_in_place = compute_commands(
            msg.linear.x, msg.angular.z)
        self._target_steering = steering_angle
        self._rotate_in_place = rotate_in_place

        # motor_rpm → tekerlek açısal hızı (rad/s)
        # Traction artık timer loop'ta yayınlanıyor (rotate_in_place bekleme mantığı orada)
        self._target_wheel_radps = (motor_rpm / TRACTION_GEAR) * (2.0 * math.pi / 60.0)

    # Steering ±90°'ye ulaştı mı tolerans kontrolü
    _STEERING_READY_TOL = 0.1  # rad (~6°)

    def _steering_control_loop(self):
        dt = 1.0 / 50.0

        # Slew rate limiter:
        #   - Normal sürüş: 1.5 rad/s (ani steering önler)
        #   - Rotate-in-place: bypass — traction zaten beklediği için hızlı dön
        if self._rotate_in_place:
            self._slewed_steering = self._target_steering
        else:
            max_delta = STEERING_SLEW_RATE * dt
            error_to_target = self._target_steering - self._slewed_steering
            self._slewed_steering += max(-max_delta, min(max_delta, error_to_target))

        # P controller: slewed hedef ile mevcut arasındaki hatadan velocity üret
        error = self._slewed_steering - self._current_steering
        vel_cmd = STEERING_KP * error
        vel_cmd = max(-STEERING_MAX_VEL, min(STEERING_MAX_VEL, vel_cmd))

        steering_msg = Float64MultiArray()
        steering_msg.data = [vel_cmd]
        self._steering_pub.publish(steering_msg)

        # ── Traction ──────────────────────────────────────────────────────────
        # Rotate-in-place: steering ±90°'ye yaklaşana kadar traction=0
        # Normal sürüş: anlık komut
        if self._rotate_in_place:
            at_90 = abs(abs(self._current_steering) - math.pi / 2) < self._STEERING_READY_TOL
            traction_cmd = self._target_wheel_radps if at_90 else 0.0
        else:
            traction_cmd = self._target_wheel_radps

        traction_msg = Float64MultiArray()
        traction_msg.data = [traction_cmd]
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
