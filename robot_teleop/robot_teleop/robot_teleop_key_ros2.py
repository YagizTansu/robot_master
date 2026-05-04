#!/usr/bin/env python3
 

from __future__ import print_function
import rclpy
import threading
import sys, select, os
from std_msgs.msg import String,Float64,Bool

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from geometry_msgs.msg import Twist
import time
from robot_interfaces.msg import Service

 
WAFFLE_MAX_LIN_VEL = 2.0
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d

w/s : linear movement (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : angular movement (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

stop when key released
ESC to quit
"""
emergency_service = False
stop_teleop = False
e = """
Communications Failed
"""

class Teleop:
    def __init__(self):
        print(msg)
        self.keys = set()
        self.status = 0
        self.target_linear_vel   = 0.0
        self.target_angular_vel  = 0.0
        self.key = ""

        self._thread = threading.Thread(target=self._read_keys, daemon=True)
        self._thread.start()

    def _read_keys(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if not ch:
                    continue

                if ch == '\x1b':
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A':   self.key = 'w'                        # up arrow
                        elif ch3 == 'B': self.key = 'x'                        # down arrow
                        elif ch3 == 'C': self.key = 'd'                        # right arrow
                        elif ch3 == 'D': self.key = 'a'                        # left arrow
                        elif ch3 == '3': sys.stdin.read(1); self.key = 'e'    # delete → e
                        elif ch3 == '5': sys.stdin.read(1); self.key = 'p'    # page up → p
                        elif ch3 == '6': sys.stdin.read(1); self.key = 'o'    # page down → o
                    else:
                        self.key = '\x1b'
                elif ch == '\r' or ch == '\n':
                    self.key = ' '   # enter → stop
                else:
                    self.key = ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input
        return input

    def get_key(self):
        k = self.key
        self.key = ""
        return k

    def on_release(self, key):
        self.key = ""
        return ""

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

    return vel

def linear_publisher(service_name ,data):
    service = Service()
    service.service_name = service_name
    service.request = data
    pub_linear.publish(service)

def cmd_raw_callback(cmd_data):
    if(stop_teleop == True and emergency_service == False):
        pub_cmd.publish(cmd_data)


def main():
    global settings, pub_cmd, pub_linear, pub_docking_status, turtlebot3_model, stop_teleop, emergency_service
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('advoard_teleop')
    pub_cmd = node.create_publisher(Twist, 'cmd_vel', 10)
    pub_linear = node.create_publisher(Service, '/services', 1) # Service publisher to /services topic
    pub_docking_status  = node.create_publisher(Bool, '/pallet_docking/cancel', 10) 
    sub_cmd_vel_raw  = node.create_subscription(Twist, 'cmd_vel_raw', cmd_raw_callback, 10) # pallet station pose coming from fleet manager
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    turtlebot3_model = "waffle"

    led_bool = False
    unlock_count = 3
    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    last_angle = 0
    tel = Teleop()
    r = node.create_rate(10) # 10hz
    try:
        print(msg)
        while(rclpy.ok()):
            key = tel.get_key()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                if (stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else :
                    print(vels(target_linear_vel, target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                if (stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else :
                    print(vels(target_linear_vel, target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                if (stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else :
                    print(vels(target_linear_vel, target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                if (stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else :
                    print(vels(target_linear_vel, target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                if (stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else :
                    print(vels(target_linear_vel, target_angular_vel))
            elif key == 'p' :
                unlock_count = 3
                stop_teleop = True
                print("Stopped teleop, press O to continue")
            elif key == 'o' :
                if unlock_count > 0:
                    unlock_count = unlock_count - 1
                if unlock_count == 0 or unlock_count == -1:
                    unlock_count = -1
                    emergency_service = False
                    stop_teleop = False
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    linear_publisher("service_emergency",False)
                    print("Resume Teleop")
                else:
                    print("Press O again " + str(unlock_count) +" times to unlcok")
            elif key == 'e' :
                unlock_count = 3
                emergency_service = True
                stop_teleop = True
                linear_publisher("service_emergency",True)
                print("Service Emergency")
            elif key == 'm' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else :
                    linear_direction = True
                    linear_publisher("service_linear_direction",True)
                    print("Linear Going Up")
            elif key == 'n' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_direction = False
                    linear_publisher("service_linear_direction",False)
                    print("Linear Going Down")
            elif key == 'l' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    led_bool = True
                    linear_publisher("service_led",False)
                    print("LED On")
            elif key == 'k' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    led_bool = False
                    linear_publisher("service_led",True)
                    print("LED Off")
            elif key == 'u' :
                if (stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_publisher("service_docking_abort", True)
                    pub_docking_status.publish(True)
                    print("Abort Docking")

            elif key == '1' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_publisher("service_pallet_start", True)
                    print("service_pallet_start")

            elif key == '2' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_publisher("service_pallet_front", True)
                    print("service_pallet_front")

            elif key == '3' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_publisher("service_pallet_inside", True)
                    print("service_pallet_inside")

            elif key == '4' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_publisher("service_pallet_out",True)
                    print("service_pallet_out")

            elif key == '5' :
                if (emergency_service == True or stop_teleop == True):
                    print("Stopped teleop, press O to continue")
                else:
                    linear_publisher("service_pallet_drop",True)
                    print("service_pallet_drop")


            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            if (stop_teleop == False and emergency_service == False):
                last_angle = control_angular_vel
                pub_cmd.publish(twist)
            elif (emergency_service == True):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub_cmd.publish(twist)
            r.sleep()
    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub_cmd.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()