import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Paths
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    
    # Map
    map_file = os.path.join(robot_navigation_dir, 'map', 'aws_warehouse.yaml')
    
    # Navigation parameters
    nav2_params_file = os.path.join(robot_navigation_dir, 'config', 'robot_move_base.yaml')
    
    # AMCL parameters
    amcl_params_file = os.path.join(robot_navigation_dir, 'config', 'amcl_config.yaml')
    
    # RViz
    rviz_config_file = os.path.join(robot_navigation_dir, 'rviz', 'robot_rviz_config.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for navigation nodes (debug, info, warn, error)'),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file to load'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use for navigation'),
        
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=amcl_params_file,
            description='Full path to the ROS2 parameters file to use for AMCL'),
        
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': LaunchConfiguration('map'),
                'topic_name': 'map',
                'frame_id': 'map'
            }]),
        
        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                amcl_params_file,
                {'use_sim_time': use_sim_time,
                 'set_initial_pose': True}
            ]
            ),
                
        # Lifecycle Manager for Map Server and AMCL
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'bond_timeout': 4.0,
                'node_names': ['map_server', 'amcl']
            }]),
        
        # AMCL Pose Saver
        Node(
            package='robot_navigation',
            executable='save_amcl_pose.py',
            name='amcl_pose_saver',
            output='screen'),
        
        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_navigation_dir, 'launch', 'custom_navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('params_file'),
                'log_level': LaunchConfiguration('log_level')
            }.items()),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        
        
        # Initial Pose Publisher
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_navigation',
                    executable='publish_initial_pose.py',
                    name='initial_pose_publisher',
                    output='screen')
            ]),
    ])
