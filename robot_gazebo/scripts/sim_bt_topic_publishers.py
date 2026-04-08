#!/usr/bin/env python3
"""Simulation-only BT topic publishers.

Publishes the topics that the idle-monitor Behaviour Tree requires but that
are NOT produced by Gazebo or Nav2 during simulation:

  /emergency_stop          std_msgs/Bool               → False (no emergency)
  /battery_status          sensor_msgs/BatteryState    → 85 % charged, discharging
  /diagnostics_agg         diagnostic_msgs/DiagnosticArray → all OK
  /fleet_manager/heartbeat std_msgs/Bool               → True (fleet online)

All values can be overridden at launch via ROS 2 parameters so that fault
scenarios can be injected without modifying this file.

Usage (standalone):
  ros2 run robot_gazebo sim_bt_topic_publishers.py

Usage with parameters (inject low battery):
  ros2 run robot_gazebo sim_bt_topic_publishers.py \
      --ros-args -p battery_percentage:=15.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SimBtTopicPublishers(Node):
    """Single node that publishes all simulation-only BT prerequisite topics."""

    def __init__(self) -> None:
        super().__init__('sim_bt_topic_publishers')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('emergency_active', False)
        self.declare_parameter('battery_percentage', 85.0)   # 0–100
        self.declare_parameter('battery_voltage', 24.0)      # V
        self.declare_parameter('fleet_online', True)
        self.declare_parameter('diagnostics_ok', True)
        self.declare_parameter('motor_temperature_celsius', 35.0)

        rate_hz: float = self.get_parameter('publish_rate_hz').value
        period_sec: float = 1.0 / rate_hz

        # ── QoS ───────────────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── Publishers ────────────────────────────────────────────────────────
        self._emergency_pub = self.create_publisher(Bool, '/emergency_stop', qos)
        self._battery_pub   = self.create_publisher(BatteryState, '/battery_status', qos)
        self._diag_pub      = self.create_publisher(DiagnosticArray, '/diagnostics_agg', qos)
        self._fleet_pub     = self.create_publisher(Bool, '/fleet_manager/heartbeat', qos)

        # ── Timer ─────────────────────────────────────────────────────────────
        self._timer = self.create_timer(period_sec, self._publish_all)

        self.get_logger().info(
            f'SimBtTopicPublishers started at {rate_hz} Hz. '
            'Set parameters to inject fault scenarios.'
        )

    # ── Publish callback ──────────────────────────────────────────────────────

    def _publish_all(self) -> None:
        now = self.get_clock().now().to_msg()

        self._publish_emergency(now)
        self._publish_battery(now)
        self._publish_diagnostics(now)
        self._publish_fleet()

    def _publish_emergency(self, stamp) -> None:
        msg = Bool()
        msg.data = bool(self.get_parameter('emergency_active').value)
        self._emergency_pub.publish(msg)

    def _publish_battery(self, stamp) -> None:
        pct: float = float(self.get_parameter('battery_percentage').value)
        voltage: float = float(self.get_parameter('battery_voltage').value)

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.voltage = voltage
        # BatteryState.percentage is normalised to 0.0–1.0
        msg.percentage = pct / 100.0
        msg.present = True
        # POWER_SUPPLY_STATUS_DISCHARGING = 2
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        # POWER_SUPPLY_HEALTH_GOOD = 2
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        # POWER_SUPPLY_TECHNOLOGY_LION = 2
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        self._battery_pub.publish(msg)

        if pct < 20.0:
            self.get_logger().warn(
                f'[SIM] Battery CRITICAL: {pct:.1f}%', throttle_duration_sec=10.0
            )
        elif pct < 30.0:
            self.get_logger().warn(
                f'[SIM] Battery LOW: {pct:.1f}%', throttle_duration_sec=10.0
            )

    def _publish_diagnostics(self, stamp) -> None:
        ok: bool = bool(self.get_parameter('diagnostics_ok').value)
        motor_temp: float = float(self.get_parameter('motor_temperature_celsius').value)

        array_msg = DiagnosticArray()
        array_msg.header.stamp = stamp

        # Motor driver status
        motor_status = DiagnosticStatus()
        motor_status.name = 'motor_driver'
        motor_status.hardware_id = 'sim_motor_driver'
        motor_status.message = 'OK' if ok else 'Simulated motor fault'
        motor_status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
        motor_status.values = [
            KeyValue(key='temperature', value=str(motor_temp)),
            KeyValue(key='current_A',   value='2.5'),
            KeyValue(key='mode',        value='sim'),
        ]
        array_msg.status.append(motor_status)

        # IMU status
        imu_status = DiagnosticStatus()
        imu_status.name = 'imu_sensor'
        imu_status.hardware_id = 'sim_imu'
        imu_status.message = 'OK'
        imu_status.level = DiagnosticStatus.OK
        array_msg.status.append(imu_status)

        # LiDAR status
        lidar_status = DiagnosticStatus()
        lidar_status.name = 'lidar'
        lidar_status.hardware_id = 'sim_lidar'
        lidar_status.message = 'OK'
        lidar_status.level = DiagnosticStatus.OK
        array_msg.status.append(lidar_status)

        self._diag_pub.publish(array_msg)

    def _publish_fleet(self) -> None:
        msg = Bool()
        msg.data = bool(self.get_parameter('fleet_online').value)
        self._fleet_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBtTopicPublishers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
