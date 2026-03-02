#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import os
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        self.pose_file = os.path.expanduser('~/.ros/last_amcl_pose.json')
        
        # 2 saniye bekle ve pozisyonu yayınla
        time.sleep(2.0)
        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        if os.path.exists(self.pose_file):
            try:
                with open(self.pose_file, 'r') as f:
                    pose_data = json.load(f)
                
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                
                msg.pose.pose.position.x = pose_data['x']
                msg.pose.pose.position.y = pose_data['y']
                msg.pose.pose.position.z = pose_data['z']
                
                msg.pose.pose.orientation.x = pose_data['qx']
                msg.pose.pose.orientation.y = pose_data['qy']
                msg.pose.pose.orientation.z = pose_data['qz']
                msg.pose.pose.orientation.w = pose_data['qw']
                
                # Covariance matrisi
                msg.pose.covariance[0] = 0.25
                msg.pose.covariance[7] = 0.25
                msg.pose.covariance[35] = 0.06853891909122467
                
                self.publisher.publish(msg)
                self.get_logger().info(f'Published initial pose: x={pose_data["x"]:.2f}, y={pose_data["y"]:.2f}')
            except Exception as e:
                self.get_logger().error(f'Failed to load/publish pose: {e}')
        else:
            self.get_logger().warn('No saved pose file found')

def main(args=None):
    rclpy.init(args=args)
    publisher = InitialPosePublisher()
    time.sleep(1.0)  # Yayınlamak için bekle
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
