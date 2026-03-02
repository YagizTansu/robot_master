from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_custom_behaviour_executor',
            executable='monitor_bt_node',
            name='monitor_bt_node',
            output='screen',
            parameters=[],
        ),
    ])
