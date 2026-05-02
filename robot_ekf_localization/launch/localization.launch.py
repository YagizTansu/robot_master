from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_ekf_localization')
    robot_slam_dir = get_package_share_directory('robot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # ── EKF Local ─────────────────────────────────────────────────────────────
    # /odom_kinco → /odometry/local + TF odom→base_footprint
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf_local.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    # ── SLAM Toolbox Localization ─────────────────────────────────────────────
    # /lidar_top/scan + warehouse_map.posegraph → TF map→odom
    # AMCL'nin yerine geçer: parçacık filtresi yerine scan-matching (Ceres)
    # Dönüşlerde çok daha kararlı: minimum_travel_heading=0.05 rad (3°) ile anlık düzeltme
    slam_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_slam_dir, 'launch', 'slam_localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_local_node,
        slam_localization_launch,
    ])
