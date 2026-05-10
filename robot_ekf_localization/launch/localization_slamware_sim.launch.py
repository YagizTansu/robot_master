"""
localization_slamware_sim.launch.py
=====================================
Simülasyon için slamware_bridge tabanlı lokalizasyon stack'i.

Gerçek robot (robot_bringup_real + localization.launch.py):
  kinco_bridge        → /odom_kinco            (motor encoder Ackermann odometri)
  slamware_bridge     → /slamware_odom          (Slamware SLAM mutlak konumu)
  ekf_local_node      → /odometry/local         (ekf_slamware.yaml)
                      → TF odom→base_footprint

Simülasyon (bu dosya):
  sim_kinco_bridge    → /odom_kinco            (joint_states Ackermann odometri)
  sim_slamware_bridge → /slamware_odom          (ground_truth + Gaussian gürültü)
  ekf_local_node      → /odometry/local         (ekf_slamware.yaml — DEĞİŞMEDEN)
                      → TF odom→base_footprint
  static TF           → map→odom (identity)     (Slamware odom≈map varsayımı)

Ön koşul: Gazebo + ground_truth_publisher çalışıyor olmalı.
          (robot_dynamic_warehouse.launch.py ile başlatılır)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ekf = get_package_share_directory('robot_ekf_localization')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon saatini kullan',
    )

    # ── sim_kinco_bridge ──────────────────────────────────────────────────────
    # Gerçek: kinco_bridge (seri port encoder) → /odom_kinco
    # Sim:    joint_states → Ackermann kinematiği → /odom_kinco
    sim_kinco_bridge_node = Node(
        package='robot_motor_controller',
        executable='sim_kinco_bridge',
        name='sim_kinco_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── sim_slamware_bridge ───────────────────────────────────────────────────
    # Gerçek: Slamware Aurora SLAM → slamware_bridge.py → /slamware_odom
    # Sim:    /ground_truth/odom   → sim_slamware_bridge → /slamware_odom
    #
    # Gürültü değerleri Slamware Aurora veri sayfasına dayanmaktadır:
    #   pozisyon hassasiyeti: ±2 cm (σ≈1 cm),  yaw hassasiyeti: ±0.5° (σ≈0.15°)
    #   Gerçekçi test için use_noise:true; ideal karşılaştırma için false yapın.
    sim_slamware_bridge_node = Node(
        package='robot_gazebo',
        executable='sim_slamware_bridge.py',
        name='sim_slamware_bridge',
        output='screen',
        parameters=[{
            'use_sim_time':   use_sim_time,
            'noise_x_std':    0.02,    # m  — Slamware Aurora ±2 cm
            'noise_y_std':    0.02,    # m
            'noise_yaw_std':  0.005,   # rad — ~0.3 derece
            'use_noise':      True,
        }],
    )

    # ── EKF Local (ekf_slamware.yaml — gerçek robotla özdeş) ─────────────────
    # odom0: /odom_kinco     [vx, vyaw — tekerlek hızları]
    # pose0: /slamware_odom  [x, y, yaw — mutlak konum]
    # → /odometry/local  +  TF odom→base_footprint
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[
            os.path.join(pkg_ekf, 'config', 'ekf_slamware.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
            # RViz "2D Pose Estimate" → EKF iç durumunu sıfırla
            # Gerçek robotda slamware_bridge.py bu komutu alarak Slamware'i
            # yeniden lokalize eder; simülasyonda doğrudan EKF'e iletiyoruz.
            ('set_pose', '/initialpose'),
        ],
    )

    # ── Static TF: map → odom (identity) ─────────────────────────────────────
    # Gerçek robotda Slamware'in iç haritası odom frame'i ile hizalıdır,
    # bu yüzden slamware_bridge.py harita konumunu frame_id="odom" olarak yayınlar.
    # Simülasyonda ground_truth map frame'indedir; statik identity TF ile
    # map ≡ odom olur ve Nav2 global planlaması çalışabilir.
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        sim_kinco_bridge_node,
        sim_slamware_bridge_node,
        ekf_local_node,
        static_map_odom_tf,
    ])
