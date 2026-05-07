"""
Relay node: subscribes to /cmd_vel (geometry_msgs/Twist) and republishes
as /tricycle_controller/cmd_vel (geometry_msgs/TwistStamped).

tricycle_controller in ROS2 Jazzy only accepts TwistStamped, while most
navigation and teleop stacks publish plain Twist to /cmd_vel.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self._pub = self.create_publisher(
            TwistStamped, '/tricycle_controller/cmd_vel', 10)
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._callback, 10)

    def _callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self._pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
