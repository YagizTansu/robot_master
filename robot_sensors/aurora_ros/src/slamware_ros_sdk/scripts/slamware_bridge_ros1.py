#!/usr/bin/env python3

import os
import rospy
import rospkg
import numpy as np
import transforms3d.quaternions as quat
import transforms3d.euler as euler
import math

from rospy.exceptions       import ROSException
from PIL                    import Image
from ruamel.yaml            import YAML
from std_msgs.msg           import String
from geometry_msgs.msg      import PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg           import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg        import Imu, MagneticField, LaserScan, Image, PointCloud2
from slamware_ros_sdk.srv   import SyncGetStcm, SyncGetStcmRequest, SyncSetStcm, SyncSetStcmRequest, RelocalizationRequest
from slamware_ros_sdk.msg   import SyncMapRequest, SetMapLocalizationRequest, SetMapUpdateRequest, ClearMapRequest, RecoverLocalizationRequest, \
                                    RectFlt32, LocalizationOptions, OptionalInt32, OptionalLocalizationMovement, LocalizationMovement,  \
                                    SystemStatus, RelocalizationCancelRequest, RelocalizationStatus

class ROSNode:
    def __init__(self):
        rospy.init_node('slamware_sdk_client')
        self.package_path = rospkg.RosPack().get_path("slamware_ros_sdk")
        self.config_path  = rospkg.RosPack().get_path("advobot_config")
        self.rate_trans   = rospy.Rate(50)  # 50 Hz
        self.map_name     = "eldor_m2"

        # Publishers
        self.pub_sync_map             = rospy.Publisher('/slamware_ros_sdk_server_node/sync_map',              SyncMapRequest,              queue_size=10)
        self.pub_clear_map            = rospy.Publisher('/slamware_ros_sdk_server_node/clear_map',             ClearMapRequest,             queue_size=10)
        self.pub_set_map_update       = rospy.Publisher('/slamware_ros_sdk_server_node/set_map_update',        SetMapUpdateRequest,         queue_size=10)
        self.pub_set_map_localization = rospy.Publisher('/slamware_ros_sdk_server_node/set_map_localization',  SetMapLocalizationRequest,   queue_size=10)
        self.pub_set_map_localization = rospy.Publisher('/slamware_ros_sdk_server_node/relocalization/cancel', RelocalizationCancelRequest, queue_size=10)
        self.pub_odom_geometry        = rospy.Publisher('/slamware_odom',                                      PoseWithCovarianceStamped,   queue_size=10)
        self.pub_odom_pose            = rospy.Publisher('/slamware_pose',                                      PoseWithCovarianceStamped,   queue_size=10)
        self.pub_scan                 = rospy.Publisher('/scan',                                               LaserScan,                   queue_size=10)
        # self.pub_set_pose             = rospy.Publisher('/slamware_ros_sdk_server_node/set_pose',              Pose,                       queue_size=10)
        # self.pub_recover_localization = rospy.Publisher('/slamware_ros_sdk_server_node/recover_localization',  RecoverLocalizationRequest, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/initialpose',                                        PoseWithCovarianceStamped, self.callback_initial_pose)
        rospy.Subscriber('/slamware_ros_sdk_server_node/scan',                  LaserScan,                 self.callback_scan)
        rospy.Subscriber('/slamware_ros_sdk_server_node/robot_pose',            PoseStamped,               self.callback_robot_pose)
        rospy.Subscriber('/slamware_ros_sdk_server_node/map_metadata',          MapMetaData,               self.callback_map_metadata)
        rospy.Subscriber('/slamware_ros_sdk_server_node/map',                   OccupancyGrid,             self.callback_map)
        rospy.Subscriber('/slamware_ros_sdk_server_node/imu_raw_data',          Imu,                       self.callback_imu_raw_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/odom',                  Odometry,                  self.callback_odom)
        rospy.Subscriber('/slamware_ros_sdk_server_node/state',                 String,                    self.callback_state)
        rospy.Subscriber('/slamware_ros_sdk_server_node/system_status',         SystemStatus,              self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/relocalization_status', RelocalizationStatus,      self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/left_image_raw',        Image,                     self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/right_image_raw',       Image,                     self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/stereo_keypoints',      Image,                     self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/point_cloud',           PointCloud2,               self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/depth_image_colorized', Image,                     self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/depth_image_raw',       Image,                     self.callback_imu_raw_mag_data)
        rospy.Subscriber('/slamware_ros_sdk_server_node/semantic_segmentation', Image,                     self.callback_imu_raw_mag_data)

        # Services
        rospy.ServiceProxy('/slamware_ros_sdk_server_node/sync_get_stcm',  SyncGetStcm)
        rospy.ServiceProxy('/slamware_ros_sdk_server_node/sync_set_stcm',  SyncSetStcm)
        rospy.ServiceProxy('/slamware_ros_sdk_server_node/relocalization', RelocalizationRequest)

        rospy.loginfo("Checking Slamware availability")
        while True:
            try:
                if rospy.is_shutdown():
                    break
                rospy.wait_for_service('/slamware_ros_sdk_server_node/sync_set_stcm', timeout = 5)
                rospy.wait_for_service('/slamware_ros_sdk_server_node/sync_get_stcm', timeout = 5)

                rospy.loginfo("Slamware is available.")
                # self.set_stcm_from_file()
                break
            except ROSException as e:
                rospy.logerr("Slamware not available. Retrying in 2 seconds...")
                rospy.sleep(2)

    def check_and_create_file(self, file_path):
        if not os.path.exists(file_path):
            # Create the file
            with open(file_path, 'w') as file:
                pass  # Creating an empty file
            print(f"File created: {file_path}")
        else:
            print(f"File already exists: {file_path}")

    def rotate_yaw_by_180(self, yaw):
        new_yaw = yaw + np.pi  # 180 degrees in radians
        # Normalize yaw to stay within -π to π
        new_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
        # Recreate the rotation matrix with the updated yaw
        new_rotation_matrix = euler.euler2mat(0, 0, new_yaw)  # Roll and pitch remain the same (0)
        # Convert the updated rotation matrix back to quaternion
        new_quaternion = quat.mat2quat(new_rotation_matrix)
        return new_quaternion
        
    def publish_sync_map(self):
        msg = SyncMapRequest()
        self.pub_sync_map.publish(msg)
        rospy.loginfo(f"Triggered sync map")

    def publish_set_pose(self, robot_pose_with_cov):
        msg = Pose()
        msg.position    = robot_pose_with_cov.pose.pose.position 
        msg.orientation = robot_pose_with_cov.pose.pose.orientation
        self.publish_recover_localization(msg.position.x, msg.position.y)
        self.pub_set_pose.publish(msg)
        rospy.loginfo(f"Set pose to lidar")

    def publish_clear_map(self):
        msg = ClearMapRequest()

        self.pub_clear_map.publish(msg)
        self.pub_clear_map.publish(msg)
        rospy.loginfo(f"Triggered clear map")

    def publish_set_map_update(self, activate):
        msg = SetMapUpdateRequest()
        msg.enabled = activate  # True to Enable

        self.pub_set_map_update.publish(msg)
        self.pub_set_map_update.publish(msg)
        rospy.loginfo(f"Mapping is set to: " + str(activate))

    def publish_set_map_localization(self, activate):
        msg = SetMapLocalizationRequest()
        msg.enabled     = activate  # True to Enable

        self.pub_set_map_localization.publish(msg)
        self.pub_set_map_localization.publish(msg)
        rospy.loginfo(f"Localization is set to: " + str(activate))

    def publish_recover_localization(self, pose_x, pose_y, width = 4.0, height = 4.0):
        msg = RecoverLocalizationRequest()

        msg.area                        = RectFlt32(pose_x, pose_y, width, height)  # x of origin, y of origin, width, height
        msg.options                     = LocalizationOptions()
        msg.options.max_time_ms         = OptionalInt32(is_valid = True, value = 2000)  # Timeout of relocalization in ms
        msg.options.mvmt_type           = OptionalLocalizationMovement()
        msg.options.mvmt_type.is_valid  = True
        msg.options.mvmt_type.value     = LocalizationMovement(type = 2)  # -1 = Unknown
                                                                    #  0 = Static
                                                                    #  1 = Rotation only
                                                                    #  2 = Any movement
        self.pub_recover_localization.publish(msg)
        rospy.loginfo(f"Recover localization.")

    def callback_initial_pose(self, msg):
        rospy.loginfo(f"Received Initial pose request.")

        rotation_matrix = quat.quat2mat([msg.pose.pose.orientation.w, 
                                         msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z])
        _, _, yaw = euler.mat2euler(rotation_matrix)
        new_yaw = self.rotate_yaw_by_180(yaw)
        msg.pose.pose.orientation.w = new_yaw[0]
        msg.pose.pose.orientation.x = new_yaw[1]
        msg.pose.pose.orientation.y = new_yaw[2]
        msg.pose.pose.orientation.z = new_yaw[3]

        distance = 0.55  # distance between center of robot and slamware in meters
        new_x = msg.pose.pose.position.x + distance * math.cos(yaw)
        new_y = msg.pose.pose.position.y + distance * math.sin(yaw)
        msg.pose.pose.position.x = new_x
        msg.pose.pose.position.y = new_y
        
        self.publish_set_pose(msg)
        
    def callback_state(self, msg):
        if (msg.data != 'connected'):
            rospy.loginfo(f"Slamware not connected! State: " + msg.data)
        
    def callback_odom(self, msg):
        geo_odom = PoseWithCovarianceStamped()
        geo_odom.header          = msg.header
        geo_odom.pose.pose       = msg.pose.pose
        geo_odom.pose.covariance = msg.pose.covariance

        # Step 1: Convert quaternion to euler angles (we only care about yaw)
        rotation_matrix = quat.quat2mat([geo_odom.pose.pose.orientation.w, 
                                         geo_odom.pose.pose.orientation.x, 
                                         geo_odom.pose.pose.orientation.y, 
                                         geo_odom.pose.pose.orientation.z])
        _, _, yaw = euler.mat2euler(rotation_matrix)
        # Step 2: Calculate new position by moving 0.55 meters forward in the direction of yaw
        distance = 0.55  # distance between center of robot and slamware in meters
        new_x = geo_odom.pose.pose.position.x + distance * math.cos(yaw)
        new_y = geo_odom.pose.pose.position.y + distance * math.sin(yaw)

        # rotates yaw, disable here if slamware is facing robot front
        new_orientation = self.rotate_yaw_by_180(yaw)
        geo_odom.pose.pose.orientation.w = new_orientation[0]
        geo_odom.pose.pose.orientation.x = new_orientation[1]
        geo_odom.pose.pose.orientation.y = new_orientation[2]
        geo_odom.pose.pose.orientation.z = new_orientation[3]

        geo_odom.pose.pose.position.x = new_x
        geo_odom.pose.pose.position.y = new_y
        geo_odom.header.frame_id = "odom"
        self.pub_odom_geometry.publish(geo_odom)

    def callback_scan(self, msg):
        msg_remap = LaserScan()
        msg_remap = msg
        msg_remap.header.frame_id = "slamware"

        self.pub_scan.publish(msg_remap)

    def callback_map(self, msg):
        rospy.loginfo(f"Received map: {msg}")

    def callback_imu_raw_data(self, msg):
        rospy.loginfo(f"Received imu_raw_data: {msg}")

    def callback_robot_pose(self, msg):
        rospy.loginfo(f"Received robot_pose: {msg}")

    def callback_map_metadata(self, msg):
        rospy.loginfo(f"Received map_metadata: {msg}")

    def extract_dimension(self, data, keyword):
        keyword_bytes = keyword.encode('utf-8')
        start_index = data.find(keyword_bytes)
        
        if start_index == -1:
            rospy.logerr(f"Could not find {keyword} in data.")
            return None

        value_start = start_index + len(keyword_bytes) + 2
        value_end   = data.find(b'\x00', value_start) - 1
        value_str   = data[value_start:value_end].decode('utf-8')

        try:
            return int(value_str)
        except ValueError:
            rospy.logerr(f"Failed to parse {keyword} value from data.")
            return None
        
    def extract_float(self, data, keyword):
        keyword_bytes = keyword.encode('utf-8')
        start_index = data.find(keyword_bytes)
        
        if start_index == -1:
            rospy.logerr(f"Could not find {keyword} in data.")
            return None

        value_start = start_index + len(keyword_bytes) + 2
        value_end   = data.find(b'\x00', value_start) - 1
        value_str   = data[value_start:value_end].decode('utf-8')

        try:
            return float(value_str)
        except ValueError:
            rospy.logerr(f"Failed to parse {keyword} value from data.")
            return None
    
    def set_pose_from_file(self):
        file_path = self.config_path + "/config/pose.txt"
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                values = [float(line.strip()) for line in file]
                msg = Pose()
                msg.position.x    = values[0] 
                msg.position.y    = values[1] 
                msg.position.z    = 0.0 

                rotation_matrix = quat.quat2mat([values[3], 
                                                 0.0, 
                                                 0.0, 
                                                 values[2]])
                _, _, yaw = euler.mat2euler(rotation_matrix)
                new_yaw = self.rotate_yaw_by_180(yaw)
                msg.orientation.w = new_yaw[0]
                msg.orientation.x = new_yaw[1]
                msg.orientation.y = new_yaw[2]
                msg.orientation.z = new_yaw[3]

                self.publish_recover_localization(msg.position.x, msg.position.y)
                self.pub_set_pose.publish(msg)
                rospy.loginfo(f"Set pose to lidar")
        
    def save_stcm_and_pgm(self, response):
        stcm_dir = self.package_path + "/maps/" + self.map_name + ".stcm"
        self.check_and_create_file(stcm_dir)

        with open(stcm_dir, 'wb') as stcm_file:
            stcm_file.write(response.raw_stcm)
        
        data = np.frombuffer(response.raw_stcm, dtype=np.uint8)

        width       = self.extract_dimension(response.raw_stcm, "dimension_width")
        height      = self.extract_dimension(response.raw_stcm, "dimension_height")
        resolution  = self.extract_float(response.raw_stcm, "resolution_x")  
        origin_x    = self.extract_float(response.raw_stcm, "origin_x")
        origin_y    = self.extract_float(response.raw_stcm, "origin_y")

        if None in (width, height, resolution, origin_x, origin_y):
            rospy.logerr("Failed to extract necessary metadata from the data.")
            return

        expected_size = width * height

        if data.size >= expected_size:
            trimmed_data = data[:expected_size]
            image_data = trimmed_data.reshape((height, width))
        else:
            rospy.logerr(f"Data size ({data.size}) is smaller than expected ({expected_size}).")
            return

        pgm_dir = self.package_path + "/maps/" + self.map_name + ".pgm"
        image = Image.fromarray(image_data, mode='L')
        image.save(pgm_dir)
        rospy.loginfo("Map saved successfully.")

        yaml_data = {
            'image': self.map_name + ".pgm",
            'resolution': resolution,
            'origin': [origin_x, origin_y, 0.0],  
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        yaml_dir = self.package_path + "/maps/" + self.map_name + ".yaml"
        yaml = YAML()
        yaml.indent(mapping=2, sequence=4, offset=2)
        yaml.default_flow_style = None
        with open(yaml_dir, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file)

        rospy.loginfo("YAML file created successfully.")

    def set_stcm_from_file(self):
        stcm_dir = self.package_path + "/maps/" + self.map_name + ".stcm"
        if os.path.exists(stcm_dir):
            with open(stcm_dir, 'rb') as stcm_file:
                raw_stcm = stcm_file.read()

            try:
                self.publish_clear_map()
                self.publish_set_map_update(False)
                self.publish_set_map_localization(False)
                sync_set_stcm = rospy.ServiceProxy('/slamware_ros_sdk_server_node/sync_set_stcm', SyncSetStcm)

                request = SyncSetStcmRequest()
                request.raw_stcm    = np.frombuffer(raw_stcm, dtype=np.uint8).tolist()
                request.robot_pose  = Pose()  # All values are default (0.0)

                request.robot_pose.position.x    = 0.0
                request.robot_pose.position.y    = 0.0
                request.robot_pose.position.z    = 0.0
                request.robot_pose.orientation.x = 0.0
                request.robot_pose.orientation.y = 0.0
                request.robot_pose.orientation.z = 0.0
                request.robot_pose.orientation.w = 1.0

                response = sync_set_stcm(request)
                self.set_pose_from_file()
                rospy.loginfo("STCM data set successfully.")

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        else:
            rospy.logerr(f"Map file doesn't exist")

    def get_map_data(self):
        try:
            sync_get_stcm = rospy.ServiceProxy('/slamware_ros_sdk_server_node/sync_get_stcm', SyncGetStcm)
            request = SyncGetStcmRequest()
            response = sync_get_stcm(request)

            # self.save_stcm_and_pgm(response)
            self.set_stcm_from_file()  # Read the STCM file and call the set service

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ROSNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
