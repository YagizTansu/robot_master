#!/usr/bin/env python3
"""
Simple battery state publisher for testing BatteryCheck BT node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        
        # Declare parameters
        self.declare_parameter('battery_percentage', 85.0)
        self.declare_parameter('publish_rate', 1.0)
        
        # Create publisher
        self.publisher_ = self.create_publisher(
            BatteryState,
            '/battery_status',
            10
        )
        
        # Get publish rate
        rate = self.get_parameter('publish_rate').value
        timer_period = 1.0 / rate
        
        # Create timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Battery Publisher started')
        
    def timer_callback(self):
        msg = BatteryState()
        
        # Get battery percentage from parameter
        percentage = self.get_parameter('battery_percentage').value
        
        # Fill message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = percentage / 100.0  # 0.0 to 1.0
        msg.voltage = 24.0  # Örnek voltage
        msg.temperature = 25.0  # Örnek sıcaklık
        msg.current = -5.0  # Negatif = şarj oluyor
        msg.charge = percentage * 100.0  # Ah cinsinden
        msg.capacity = 100.0  # Total capacity
        msg.design_capacity = 100.0
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery: {percentage:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    
    battery_publisher = BatteryPublisher()
    
    try:
        rclpy.spin(battery_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        battery_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
