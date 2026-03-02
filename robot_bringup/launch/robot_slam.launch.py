from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get the package directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # SLAM Toolbox parameters file path
    slam_params_file = os.path.join(robot_navigation_dir, 'config', 'slam_toolbox_params.yaml')
    
    # RViz config file
    rviz_config_file = os.path.join(robot_navigation_dir, 'rviz', 'robot_slam_config.rviz')
    

   
    # SLAM Toolbox Node (async mode for online mapping)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
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
    
    # Lifecycle Manager for SLAM Toolbox
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bond_timeout': 4.0,
            'node_names': ['slam_toolbox']
        }]
    )
    

    # Add all actions
    ld.add_action(slam_toolbox_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)
    
    return ld
