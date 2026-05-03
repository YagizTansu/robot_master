from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_ekf_localization')

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

    # ── AMCL Localization ─────────────────────────────────────────────────────
    # /lidar_top/scan + /map → TF map→odom
    # Particle filter tabanlı: rotation dahil her harekette güncellenir
    # update_min_a=0.05 rad (3°) → yerinde dönüşlerde bile anlık düzeltme
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'amcl.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ekf_local_node,
        amcl_node,
        amcl_lifecycle_manager,
    ])
