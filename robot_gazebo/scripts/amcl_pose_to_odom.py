#!/usr/bin/env python3
"""
AMCL Pose → Odometry Converter
================================
Converts geometry_msgs/PoseWithCovarianceStamped published by AMCL on
/amcl_pose into nav_msgs/Odometry on /amcl_pose_as_odom so the standard
localization_benchmark node can compare it against ground truth.

The child_frame_id of the output Odometry is set to 'base_footprint'
and the header frame_id is 'map', matching AMCL's coordinate convention.

Usage (standalone):
  ros2 run robot_gazebo amcl_pose_to_odom.py

Launch argument override:
  ros2 run robot_gazebo amcl_pose_to_odom.py --ros-args \
      -p input_topic:=/amcl_pose \
      -p output_topic:=/amcl_pose_as_odom
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclPoseToOdom(Node):

    def __init__(self) -> None:
        super().__init__("amcl_pose_to_odom")

        self.declare_parameter("input_topic",  "/amcl_pose")
        self.declare_parameter("output_topic", "/amcl_pose_as_odom")

        in_topic  = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self._pub = self.create_publisher(Odometry, out_topic, 50)
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            in_topic,
            self._cb,
            50,
        )

        self.get_logger().info(
            f"amcl_pose_to_odom: {in_topic} → {out_topic}"
        )

    def _cb(self, msg: PoseWithCovarianceStamped) -> None:
        odom = Odometry()
        odom.header            = msg.header          # frame_id = "map"
        odom.child_frame_id    = "base_footprint"
        odom.pose              = msg.pose             # PoseWithCovariance
        # twist is left zero – benchmark only uses pose fields
        self._pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclPoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
