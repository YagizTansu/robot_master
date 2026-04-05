import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_slam_dir = get_package_share_directory('robot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            robot_slam_dir, 'config', 'slam_toolbox_localization.yaml'
        ),
        description='Full path to the slam_toolbox localization parameters file',
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(robot_slam_dir, 'maps', 'map'),
        description='Full path to the map file (without .yaml / .posegraph extension)',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_map_file,
        slam_node,
    ])
