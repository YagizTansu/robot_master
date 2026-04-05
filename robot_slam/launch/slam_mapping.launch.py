import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_slam_dir = get_package_share_directory('robot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            robot_slam_dir, 'config', 'slam_toolbox_mapping.yaml'
        ),
        description='Full path to the slam_toolbox mapping parameters file',
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(
            robot_slam_dir, 'config', 'ekf_slam.yaml'
        ),
        description='Full path to the robot_localization EKF parameters file',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with slam_toolbox plugin',
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            robot_slam_dir, 'rviz', 'slam_mapping.rviz'
        ),
        description='Full path to the RViz config file',
    )

    # EKF: /odometry → odom → base_footprint TF
    # slam_toolbox requires this TF to be available before it starts
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # async_slam_toolbox_node is a lifecycle node — lifecycle_manager
    # handles configure → activate transitions automatically
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_ekf_params_file,
        declare_use_rviz,
        declare_rviz_config_file,
        ekf_node,
        slam_node,
        lifecycle_manager,
        rviz_node,
    ])
