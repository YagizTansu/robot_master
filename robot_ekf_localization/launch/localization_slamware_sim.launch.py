"""
localization_slamware_sim.launch.py
=====================================
Simülasyon lokalizasyon stack'i — Gazebo ground truth tabanlı (sıfır hata).

Mimari:
  ground_truth_publisher  → /ground_truth/odom  (Gazebo fizik motoru, sıfır gürültü)
  sim_slamware_bridge     → /slamware_odom       (PoseWithCovarianceStamped, use_noise=false)
  sim_kinco_bridge        → /odom_kinco          (joint_states → Ackermann odometri)
  EKF (odom0+pose0)       → odom→base_footprint TF + /odometry/local
  static TF               → map→odom = identity  (map≡odom)

Sonuç: Nav2 + diğer bileşenler için mükemmel, sıfır hatalı lokalizasyon.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ekf    = get_package_share_directory('robot_ekf_localization')
    pkg_gazebo = get_package_share_directory('robot_gazebo')
    _ = pkg_gazebo  # noqa: used implicitly via Node(package=...)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon saatini kullan',
    )

    # ── ground_truth_publisher ────────────────────────────────────────────────
    # Gazebo /world/lab_new/dynamic_pose/info → /ground_truth/odom
    ground_truth_node = Node(
        package='robot_gazebo',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name':   'boa_new_urdf',
            'world_name':   'lab_new',
            'map_frame':    'map',
        }],
    )

    # ── sim_slamware_bridge ───────────────────────────────────────────────────
    # /ground_truth/odom → /slamware_odom (gürültüsüz — mükemmel lokalizasyon)
    sim_slamware_node = Node(
        package='robot_gazebo',
        executable='sim_slamware_bridge.py',
        name='sim_slamware_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_noise':    False,
        }],
    )

    # ── sim_kinco_bridge ──────────────────────────────────────────────────────
    # joint_states → Ackermann kinematiği → /odom_kinco
    sim_kinco_bridge_node = Node(
        package='robot_motor_controller',
        executable='sim_kinco_bridge',
        name='sim_kinco_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── static TF: map → odom (identity) ─────────────────────────────────────
    # ground_truth + EKF pose0 sayesinde odom≡map; navigasyon map frame'de çalışır
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── EKF (odom + absolute pose) ────────────────────────────────────────────
    # odom0: /odom_kinco (hız)  +  pose0: /slamware_odom (mutlak konum GT)
    # → odom→base_footprint TF + /odometry/local
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[
            os.path.join(pkg_ekf, 'config', 'ekf_sim_odom.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        ground_truth_node,
        sim_slamware_node,
        sim_kinco_bridge_node,
        static_map_odom_tf,
        ekf_local_node,
    ])
