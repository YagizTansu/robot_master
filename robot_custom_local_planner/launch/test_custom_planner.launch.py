import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file to test custom local planner with Nav2 controller server
    """
    
    # Get the package directory
    pkg_dir = get_package_share_directory('robot_custom_local_planner')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'custom_local_planner_params.yaml')
    
    # Controller server node
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/cmd_vel', '/robot/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        controller_server
    ])
