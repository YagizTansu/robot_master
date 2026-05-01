import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 2.0    # m/s  (~3.6 km/h, forklift max)
MAX_ANG_VEL = 1.57    # rad/s yaw rate
# NOT: kinco_drive_node formülü: steering_angle = atan2(wz * wheelbase, vx)
# Dar dönüş için düşük hız gerekir! Wheelbase=1.0957m
# 0.3 m/s + 1.57 rad/s → R≈0.21m  |  2.0 m/s + 1.57 rad/s → R≈1.27m

LIN_VEL_STEP_SIZE = 0.1   # her basışta 0.1 m/s hız değişimi
ANG_VEL_STEP_SIZE = 0.1   # (artık kullanılmıyor, hold-to-steer)

msg = """
Robot Teleop - Kinco Drive
--------------------------
Moving around:
        w
   a    s    d
        x

w : ileri hız artır (+0.1 m/s)
x : hız azalt / geri (-0.1 m/s)
a : sola dön  (-0.2 rad/s, her bastıkta artar)
d : sağa dön (+0.2 rad/s, her bastıkta artar)
s / space : tam dur
q : çıkış

NOT: Dar dönüş için düşük hızda a/d'ye bas!
     steering = atan2(wz * 1.0957, vx)  →  yavaş gidince R küçülür
--------------------------
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(lin, ang):
    print('linear: {0:.2f} m/s  |  angular: {1:.2f} rad/s'.format(lin, ang))


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output


def constrain(val, low, high):
    return max(low, min(high, val))


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('robot_teleop_key')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        print(msg)
        while True:
            key = get_key(settings)

            if key == 'w':
                target_linear_velocity = constrain(
                    target_linear_velocity + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
                print_vels(target_linear_velocity, target_angular_velocity)

            elif key == 'x':
                target_linear_velocity = constrain(
                    target_linear_velocity - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
                print_vels(target_linear_velocity, target_angular_velocity)

            elif key == 'a':
                # Her bastıkta angular hızı artır (sola)
                target_angular_velocity = constrain(
                    target_angular_velocity + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
                print_vels(target_linear_velocity, target_angular_velocity)

            elif key == 'd':
                # Her bastıkta angular hızı azalt (sağa)
                target_angular_velocity = constrain(
                    target_angular_velocity - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
                print_vels(target_linear_velocity, target_angular_velocity)

            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_linear_velocity = 0.0
                control_angular_velocity = 0.0
                print_vels(0.0, 0.0)

            elif key == '\x03' or key == 'q':
                break

            else:
                # Timeout (önce hold-to-steer vardı, artık hiçbir şey yapma)
                pass

            # Her iki hızı da yumuşat
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                LIN_VEL_STEP_SIZE / 2.0)

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                ANG_VEL_STEP_SIZE / 2.0)

            twist = Twist()
            twist.linear.x = control_linear_velocity
            twist.angular.z = control_angular_velocity
            pub.publish(twist)

    except Exception as ex:
        print(ex)

    finally:
        twist = Twist()
        pub.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
