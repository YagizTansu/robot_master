#!/usr/bin/env python3
"""
Ground Truth Publisher for Localization Benchmarking
=====================================================
Extracts the robot's true pose directly from the Gazebo physics engine
via the /world/default/dynamic_pose/info bridge (gz.msgs.Pose_V → TFMessage).

Published Topics:
  /ground_truth/odom  (nav_msgs/Odometry)      – for algorithm comparison
  /ground_truth/pose  (geometry_msgs/PoseStamped) – for RViz overlay

The "world" frame in Gazebo ≡ "map" frame in ROS 2 (fixed inertial origin).
No sensor noise, no drift – 100% accurate position from the simulator.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseStamped,
    TransformStamped,
    Quaternion,
    Point,
    Vector3,
)
from builtin_interfaces.msg import Time

# The Gazebo model name as declared in model.sdf: <model name='robot'>
ROBOT_MODEL_NAME = "robot"

# Gazebo "world" frame maps to ROS "map" frame (inertial, fixed origin)
MAP_FRAME = "map"
GROUND_TRUTH_FRAME = "ground_truth_base_link"


class GroundTruthPublisher(Node):
    """
    Bridges the Gazebo dynamic pose vector to clean ROS 2 ground truth topics.

    The gz→ROS bridge publishes every model's world-frame pose as a
    TFMessage on /gz/dynamic_pose.  This node filters the robot entry and
    re-publishes it in formats convenient for localization comparison
    (nav_msgs/Odometry and geometry_msgs/PoseStamped).
    """

    def __init__(self) -> None:
        super().__init__("ground_truth_publisher")

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter("robot_name", ROBOT_MODEL_NAME)
        self.declare_parameter("map_frame", MAP_FRAME)
        self.declare_parameter("ground_truth_child_frame", GROUND_TRUTH_FRAME)
        self.declare_parameter("publish_rate_warn_hz", 0.2)  # log warn if silent > 5 s

        self._robot_name: str = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self._map_frame: str = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self._child_frame: str = (
            self.get_parameter("ground_truth_child_frame")
            .get_parameter_value()
            .string_value
        )

        # ── Subscribers ────────────────────────────────────────────────────
        # /gz/dynamic_pose is the remapped bridge topic for
        # /world/default/dynamic_pose/info (gz.msgs.Pose_V → TFMessage)
        self._tf_sub = self.create_subscription(
            TFMessage,
            "/gz/dynamic_pose",
            self._on_gz_tf,
            qos_profile=10,
        )

        # ── Publishers ─────────────────────────────────────────────────────
        # nav_msgs/Odometry – standard format for localization comparison
        self._odom_pub = self.create_publisher(
            Odometry,
            "/ground_truth/odom",
            qos_profile=10,
        )

        # geometry_msgs/PoseStamped – easy to plot in RViz as an arrow/path
        self._pose_pub = self.create_publisher(
            PoseStamped,
            "/ground_truth/pose",
            qos_profile=10,
        )

        # ── State ──────────────────────────────────────────────────────────
        self._msg_count: int = 0
        self._watchdog = self.create_timer(5.0, self._watchdog_cb)
        self._last_stamp: Time | None = None

        self.get_logger().info(
            f"GroundTruthPublisher started — tracking model '{self._robot_name}'\n"
            f"  Subscribing : /gz/dynamic_pose  (TFMessage)\n"
            f"  Publishing  : /ground_truth/odom  (nav_msgs/Odometry)\n"
            f"  Publishing  : /ground_truth/pose  (geometry_msgs/PoseStamped)\n"
            f"  Map frame   : {self._map_frame}"
        )

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _on_gz_tf(self, msg: TFMessage) -> None:
        """
        Called for every TFMessage from the Gazebo dynamic pose bridge.
        The message contains one TransformStamped per dynamic model/link.
        We search for the entry whose child_frame_id matches our robot model.
        """
        robot_tf: TransformStamped | None = None

        for transform in msg.transforms:
            # Gazebo Harmonic names the Pose_V entries after the model name
            # (not individual links).  The child_frame_id will be "robot".
            if transform.child_frame_id == self._robot_name:
                robot_tf = transform
                break

        if robot_tf is None:
            return  # robot not yet spawned or name mismatch

        self._msg_count += 1
        stamp = robot_tf.header.stamp

        # ── Publish Odometry ────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._map_frame          # map (inertial world origin)
        odom.child_frame_id = self._child_frame          # ground_truth_base_link

        t = robot_tf.transform.translation
        r = robot_tf.transform.rotation

        odom.pose.pose.position = Point(x=t.x, y=t.y, z=t.z)
        odom.pose.pose.orientation = Quaternion(x=r.x, y=r.y, z=r.z, w=r.w)

        # Covariance is zero for perfect ground truth
        odom.pose.covariance = [0.0] * 36

        # Twist is left as zero (velocity ground truth can be added later)
        odom.twist.covariance = [0.0] * 36

        self._odom_pub.publish(odom)

        # ── Publish PoseStamped ─────────────────────────────────────────────
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self._map_frame
        ps.pose.position = Point(x=t.x, y=t.y, z=t.z)
        ps.pose.orientation = Quaternion(x=r.x, y=r.y, z=r.z, w=r.w)

        self._pose_pub.publish(ps)

        self._last_stamp = stamp

    def _watchdog_cb(self) -> None:
        """Periodically log status so the user knows the node is alive."""
        if self._msg_count == 0:
            self.get_logger().warn(
                "No ground truth poses received yet.  "
                "Check that Gazebo is running and the bridge is active.\n"
                "  Expected bridge topic : /gz/dynamic_pose\n"
                f"  Expected child_frame  : {self._robot_name}"
            )
        else:
            self.get_logger().info(
                f"[ground_truth] Publishing OK — {self._msg_count} messages received."
            )
            self._msg_count = 0  # reset counter for next window


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
