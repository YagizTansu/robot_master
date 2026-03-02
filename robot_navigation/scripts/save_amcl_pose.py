#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import os

class AMCLPoseSaver(Node):
    def __init__(self):
        super().__init__('amcl_pose_saver')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.pose_file = os.path.expanduser('~/.ros/last_amcl_pose.json')
        self.last_pose = None
        
        # Her 5 saniyede bir kaydet
        self.timer = self.create_timer(5.0, self.save_pose)
        
    def pose_callback(self, msg):
        self.last_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w
        }
    
    def save_pose(self):
        if self.last_pose:
            try:
                os.makedirs(os.path.dirname(self.pose_file), exist_ok=True)
                with open(self.pose_file, 'w') as f:
                    json.dump(self.last_pose, f)
            except Exception as e:
                self.get_logger().error(f'Failed to save pose: {e}')

def main(args=None):
    rclpy.init(args=args)
    saver = AMCLPoseSaver()
    rclpy.spin(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
