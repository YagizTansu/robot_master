from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    resolution = LaunchConfiguration('resolution')
    
    # Cartographer configuration file path
    cartographer_config_dir = os.path.join(robot_navigation_dir, 'config')
    configuration_basename = 'cartographer_2d.lua'
    
    # RViz config file
    rviz_config_file = os.path.join(robot_navigation_dir, 'rviz', 'robot_cartographer_config.rviz')
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid'))
    
    ld.add_action(DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period'))
    
    # Cartographer Node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan_1', '/sick_lidar0/scan'),
            ('scan_2', '/sick_lidar1/scan'),
            ('imu', '/imu'),
            ('odom', '/odometry')
        ],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ]
    )
    
    # Include occupancy grid launch file
    occupancy_grid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_dir, 'launch', 'robot_occupancy_grid.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }.items()
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Add all actions
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_launch)
    ld.add_action(rviz_node)
    
    return ld
