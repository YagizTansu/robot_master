#!/usr/bin/env python3

import os
import math
import time
import rclpy

import numpy as np
import transforms3d.quaternions as quat
import transforms3d.euler as euler

from ament_index_python.packages import get_package_share_directory

from rclpy.node        import Node
from rclpy.qos         import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy
from std_msgs.msg      import String
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg      import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg   import Imu, LaserScan, Image as RosImage, PointCloud2

from slamware_ros_sdk.srv import SyncGetStcm, SyncSetStcm, RelocalizationRequest
from slamware_ros_sdk.msg import (SyncMapRequest,
                                  SetMapUpdateRequest,
                                  SetMapLocalizationRequest,
                                  ClearMapRequest,
                                  SystemStatus,
                                  RelocalizationCancelRequest,
                                  RelocalizationStatus)

class ROSNode(Node):

    EXPECTED_MAP_CHANGE_SEQUENCE = [
        "DeviceInited",
        "DeviceMapCleared",
        "DeviceMapLoadingCompleted",
        "DeviceInited",
        "DeviceOptimizationCompleted"
    ]

    EXPECTED_RELOCATION_SEQUENCE = [
        "DeviceRelocRunning",
        "RelocalizationRunning",
        "DeviceRelocSuccess",
        "RelocalizationSucceed",
        "RelocalizationNone"
    ]

    QOS_PROFILE = QoSProfile(
                  reliability = QoSReliabilityPolicy.RELIABLE,
                  durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
                  history     = HistoryPolicy.KEEP_LAST,
                  depth       = 1
    )

    def __init__(self):
        super().__init__('slamware_sdk_client')

        self.declare_parameter("slamware_new_map", False)
        slamware_new_map = self.get_parameter("slamware_new_map").get_parameter_value().bool_value

        self.map_name = "lab_last"
        self.package_path = get_package_share_directory("slamware_ros_sdk")
        self.map_path = self.package_path + "/maps/" + self.map_name + ".stcm" 

        self.device_status = " "
        self.relocation_status = " "

        self.new_map = OccupancyGrid()

        # Publishers
        self.pub_set_map_localization = self.create_publisher(SetMapLocalizationRequest,   '/slamware_ros_sdk_server_node/set_map_localization',10)
        self.pub_set_map_update       = self.create_publisher(SetMapUpdateRequest,         '/slamware_ros_sdk_server_node/set_map_update',10)
        self.pub_clear_map            = self.create_publisher(ClearMapRequest,             '/slamware_ros_sdk_server_node/clear_map',10)
        self.pub_sync_map             = self.create_publisher(SyncMapRequest,              '/slamware_ros_sdk_server_node/sync_map',10)
        self.pub_odom_with_cov        = self.create_publisher(PoseWithCovarianceStamped,   '/slamware_odom',10)
        self.pub_scan                 = self.create_publisher(LaserScan,                   '/scan',10)
        self.relocalization_cancel    = self.create_publisher(RelocalizationCancelRequest, '/slamware_ros_sdk_server_node/relocalization/cancel',10)
        self.pub_odom_pose            = self.create_publisher(PoseStamped,                 '/slamware_pose',10)
        self.pub_imu                  = self.create_publisher(Imu,                         '/slamware_imu',10)
        self.pub_map_with_qos         = self.create_publisher(OccupancyGrid,               '/slamware_map',self.QOS_PROFILE)

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',                                        self.callback_initial_pose,10)
        self.create_subscription(LaserScan,                 '/slamware_ros_sdk_server_node/scan',                  self.callback_scan,10)
        self.create_subscription(Odometry,                  '/slamware_ros_sdk_server_node/odom',                  self.callback_odom,10)
        self.create_subscription(String,                    '/slamware_ros_sdk_server_node/state',                 self.callback_state,10)
        self.create_subscription(SystemStatus,              '/slamware_ros_sdk_server_node/system_status',         self.callback_system_status,10)
        self.create_subscription(RelocalizationStatus,      '/slamware_ros_sdk_server_node/relocalization_status', self.callback_relocalization_status,10)
        self.create_subscription(PoseStamped,               '/slamware_ros_sdk_server_node/robot_pose',            self.callback_robot_pose,10)
        self.create_subscription(Imu,                       '/slamware_ros_sdk_server_node/imu_raw_data',          self.callback_imu_raw_data,10)
        self.create_subscription(OccupancyGrid,             '/slamware_ros_sdk_server_node/map',                   self.callback_map,10)
        # self.create_subscription(MapMetaData,               '/slamware_ros_sdk_server_node/map_metadata',          self.callback_map_metadata,10)
        # self.create_subscription(RosImage,                  '/slamware_ros_sdk_server_node/left_image_raw',        self.callback_left_image,10)
        # self.create_subscription(RosImage,                  '/slamware_ros_sdk_server_node/right_image_raw',       self.callback_right_image,10)
        # self.create_subscription(RosImage,                  '/slamware_ros_sdk_server_node/stereo_keypoints',      self.callback_stereo,10)
        # self.create_subscription(PointCloud2,               '/slamware_ros_sdk_server_node/point_cloud',           self.callback_pointcloud,10)
        # self.create_subscription(RosImage,                  '/slamware_ros_sdk_server_node/depth_image_colorized', self.callback_image_color,10)
        # self.create_subscription(RosImage,                  '/slamware_ros_sdk_server_node/depth_image_raw',       self.callback_image_raw,10)
        # self.create_subscription(RosImage,                  '/slamware_ros_sdk_server_node/semantic_segmentation', self.callback_semantic,10)

        # Services
        self.sync_get_stcm  = self.create_client(SyncGetStcm,           '/slamware_ros_sdk_server_node/sync_get_stcm')
        self.sync_set_stcm  = self.create_client(SyncSetStcm,           '/slamware_ros_sdk_server_node/sync_set_stcm')
        self.relocalization = self.create_client(RelocalizationRequest, '/slamware_ros_sdk_server_node/relocalization')

        self.get_logger().info("ROS2 node initialized")

        self.get_logger().info("Checking Slamware availability")
        self.wait_for_slamware()

        if slamware_new_map:
            self.send_stcm(self.map_path)
        else:
            self.publish_stop_mapping(True)

    def publish_sync_map(self):
        msg = SyncMapRequest()
        self.pub_sync_map.publish(msg)
        self.get_logger().info("Triggered sync map")

    def publish_clear_map(self):
        msg = ClearMapRequest()
        self.pub_clear_map.publish(msg)
        self.pub_clear_map.publish(msg)
        self.get_logger().info("Triggered clear map")

    def publish_activate_mapping(self, enable):
        msg = SetMapUpdateRequest()
        msg.enabled = enable
        self.pub_set_map_update.publish(msg)
        self.get_logger().info(f"Mapping is set to: " + str(enable))

    def publish_stop_mapping(self, enable):
        msg = SetMapLocalizationRequest()
        msg.enabled = enable
        self.pub_set_map_localization.publish(msg)
        self.get_logger().info(f"Localization is set to: " + str(enable))

    def publish_recover_localization(self):
        reloc_cancel = RelocalizationCancelRequest()
        self.relocalization_cancel.publish(reloc_cancel)
        time.sleep(1.0)

        request = RelocalizationRequest.Request()
        self.relocalization.call_async(request)

        self.get_logger().info(f"Recover localization.")
        self.wait_for_relocalization()

    # Callbacks 
    def callback_state(self, msg):
        if msg.data != 'connected':
            self.get_logger().info("Slamware not connected: " + msg.data)

    def callback_system_status(self, msg):
        self.device_status = msg.status
        if not (self.device_status == "DeviceRunning" or self.device_status == "DeviceMapCleared") :
            self.get_logger().info(self.device_status)

    def callback_relocalization_status(self, msg):
        self.relocation_status = msg.status
        if self.relocation_status != "RelocalizationNone":
            self.get_logger().info(self.relocation_status)

    def callback_initial_pose(self, msg):
        self.get_logger().info("initial pose received")
        self.publish_recover_localization()

    def callback_map(self, msg):
        self.new_map = msg
        self.new_map.header.stamp = self.get_clock().now().to_msg()
        self.pub_map_with_qos.publish(self.new_map)

    def callback_scan(self, msg):
        msg_remap = LaserScan()
        msg_remap = msg
        msg_remap.header.frame_id = "slamware"
        self.pub_scan.publish(msg_remap)

    def callback_robot_pose(self, msg):
        geo_pose = PoseStamped()
        geo_pose = msg
        geo_pose.header.frame_id = "slamware_pose"
        self.pub_odom_pose.publish(geo_pose)

    def callback_imu_raw_data(self, msg):
        imu_remap = Imu()
        imu_remap = msg
        imu_remap.header.frame_id = "slamware_imu"
        self.pub_imu.publish(imu_remap)

    def callback_odom(self, msg):
        geo_odom = PoseWithCovarianceStamped()
        geo_odom          = msg
        geo_odom.header.frame_id = "lidar_top"
        # Step 1: Convert quaternion to euler angles (we only care about yaw)
        # rotation_matrix = quat.quat2mat([geo_odom.pose.pose.orientation.w, 
        #                                  geo_odom.pose.pose.orientation.x, 
        #                                  geo_odom.pose.pose.orientation.y, 
        #                                  geo_odom.pose.pose.orientation.z])
        # _, _, yaw = euler.mat2euler(rotation_matrix)
        # Step 2: Calculate new position by moving 0.55 meters forward in the direction of yaw
        # forward_distance = - 0.85  # distance between center of robot and slamware in meters
        # sideway_distance = 0.0  # distance between center of robot and slamware in meters
        # new_x = geo_odom.pose.pose.position.x + forward_distance * math.cos(yaw) - sideway_distance * math.sin(yaw)
        # new_y = geo_odom.pose.pose.position.y + forward_distance * math.sin(yaw) + sideway_distance * math.cos(yaw)

        # rotates yaw, disable here if slamware is facing robot front
        # new_orientation = self.rotate_yaw_by_180(yaw)
        # geo_odom.pose.pose.orientation.w = new_orientation[0]
        # geo_odom.pose.pose.orientation.x = new_orientation[1]
        # geo_odom.pose.pose.orientation.y = new_orientation[2]
        # geo_odom.pose.pose.orientation.z = new_orientation[3]

        # geo_odom.pose.pose.position.x = new_x
        # geo_odom.pose.pose.position.y = new_y
        self.pub_odom_with_cov.publish(geo_odom)

    def callback_left_image(self, msg):
        self.get_logger().info("image left received")
    
    def callback_right_image(self, msg):
        self.get_logger().info("image right received")

    def callback_stereo(self, msg):
        self.get_logger().info("stereo image received")

    def callback_image_color(self, msg):
        self.get_logger().info("image color received")

    def callback_image_raw(self, msg):
        self.get_logger().info("image raw received")

    def callback_semantic(self, msg):
        self.get_logger().info("semantic segmentation received")

    def callback_pointcloud(self, msg):
        self.get_logger().info("pointcloud received")

    def callback_map_metadata(self, msg):
        self.get_logger().info("map_metadata received")

    # Utilities
    def check_and_create_file(self, file_path):
        if not os.path.exists(file_path):
            # Create the file
            with open(file_path, 'w') as file:
                pass  # Creating an empty file
            self.get_logger().info(f"File created: {file_path}")
        else:
            self.get_logger().info(f"File already exists: {file_path}")

    def rotate_yaw_by_180(self, yaw):
        new_yaw = yaw + np.pi  # 180 degrees in radians
        # Normalize yaw to stay within -π to π
        new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
        # Recreate the rotation matrix with the updated yaw
        new_rotation_matrix = euler.euler2mat(0, 0, new_yaw)  # Roll and pitch remain the same (0)
        # Convert the updated rotation matrix back to quaternion
        new_quaternion = quat.mat2quat(new_rotation_matrix)
        return new_quaternion

    def wait_for_slamware(self):
        while rclpy.ok():
            try:
                get_ready = self.sync_get_stcm.wait_for_service(timeout_sec=5.0)
                set_ready = self.sync_set_stcm.wait_for_service(timeout_sec=5.0)
                loc_ready = self.relocalization.wait_for_service(timeout_sec=5.0)

                if set_ready and get_ready and loc_ready:
                    self.get_logger().info("Slamware is available.")
                    break

            except Exception as e:
                self.get_logger().error(f"Slamware not available. Retrying in 2 seconds... ({e})")

            time.sleep(2.0)

    def send_stcm(self, path):
        if not os.path.exists(path):
            self.get_logger().error("STCM file not found")
            return

        request = SyncSetStcm.Request()
        request.mapfile = path

        set_stcm_service = self.sync_set_stcm.call_async(request)
        rclpy.spin_until_future_complete(self, set_stcm_service)

        if set_stcm_service.result() is not None:
            res = set_stcm_service.result()
            self.get_logger().info(
                f"Success: {res.success}, message: {res.message}"
            )
        else:
            self.get_logger().error("Service call failed")
        
        self.wait_for_map_change()

    def receive_stcm(self, path):
        if os.path.exists(path):
            self.get_logger().error("STCM file already exists")
            return
        
        request = SyncGetStcm.Request()
        request = path
        get_stcm_service = self.sync_get_stcm.call_async(request)

    def wait_for_map_change(self, timeout_sec=60.0, final_wait_sec=10.0):

        start_time = time.time()
        index = 0
        last_step_time = None  # when we reached second-to-last state
        sequence = self.EXPECTED_MAP_CHANGE_SEQUENCE
        warned_final_missing = False
        
        while time.time() - start_time < timeout_sec:
            current = self.device_status

            if current is not None:
                # Advance forward if possible
                for i in range(index, len(sequence)):
                    if current == sequence[i]:
                        index = i + 1

                        # If we just reached second-to-last step
                        if index == len(sequence) - 1:
                            last_step_time = time.time()
                        break

            # ✅ Completed everything
            if index >= len(sequence):
                self.get_logger().info("Map change sequence completed")
                self.publish_stop_mapping(True)
                return True

            # ⚠️ Stuck before final step
            if index == len(sequence) - 1 and last_step_time is not None:
                if time.time() - last_step_time > final_wait_sec:
                    if not warned_final_missing:
                        self.get_logger().warn("Cannot find location, move the robot")
                        warned_final_missing = True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn("Map change sequence timeout")
        return False
    
    def wait_for_relocalization(self, timeout_sec=5.0):
        start = time.time()
        sequence = self.EXPECTED_RELOCATION_SEQUENCE
        index = 0

        while time.time() - start < timeout_sec:
            current = self.relocation_status

            if current is not None:
                # allow skipping missed states safely
                while index < len(sequence) and current != sequence[index]:
                    index += 1

                if index < len(sequence) and current == sequence[index]:
                    index += 1

                if index >= len(sequence):
                    return True

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

def main():
    rclpy.init()
    node = ROSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
