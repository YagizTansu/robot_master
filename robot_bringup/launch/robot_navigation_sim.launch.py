"""
robot_navigation_sim.launch.py
================================
Simülasyonda gerçek robot ile özdeş Slamware tabanlı lokalizasyon + Nav2.

Sistem mimarisi (robot_navigation.launch.py ile bire bir):

  ┌─ Gazebo Sim ──────────────────────────────────────────────────┐
  │  gz sim → ros_gz_bridge → /joint_states, /odometry, /imu ... │
  │  ground_truth_publisher   → /ground_truth/odom                │
  └───────────────────────────────────────────────────────────────┘
          │                          │
          ▼                          ▼
  sim_kinco_bridge           sim_slamware_bridge
  /joint_states              /ground_truth/odom
       │                           │ + Gaussian gürültü
       ▼                           ▼
  /odom_kinco             /slamware_odom
  (Odometry)              (PoseWithCovarianceStamped)
          │                          │
          └──────── EKF ─────────────┘
               ekf_slamware.yaml
               (gerçek robotla özdeş)
                      │
                      ▼
               /odometry/local
               TF odom→base_footprint
                      │
          static TF map→odom (identity)
                      │
                      ▼
                   Nav2 Stack
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_ekf_loc_dir    = get_package_share_directory('robot_ekf_localization')
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    try:
        robot_slam_dir = get_package_share_directory('robot_slam')
    except PackageNotFoundError:
        robot_slam_dir = None

    # ── Launch argument ───────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gazebo simülasyon saatini kullan',
    ))

    # ── Slamware-benzeri lokalizasyon (sim) ───────────────────────────────────
    # sim_kinco_bridge + sim_slamware_bridge + EKF(ekf_slamware.yaml) + static TF
    localization_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_ekf_loc_dir, 'launch', 'localization_slamware_sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── Nav2 Navigation Stack ─────────────────────────────────────────────────
    nav2_params_file = os.path.join(
        robot_navigation_dir, 'config', 'robot_move_base.yaml'
    )
    custom_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'custom_navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
        }.items(),
    )

    # ── Map Server ────────────────────────────────────────────────────────────
    if robot_slam_dir and os.path.exists(
            os.path.join(robot_slam_dir, 'maps', 'warehouse_map.yaml')):
        map_file = os.path.join(robot_slam_dir, 'maps', 'lab_new.yaml')
    else:
        map_file = os.path.join(robot_navigation_dir, 'map', 'aws_warehouse.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'yaml_filename': map_file,
            'topic_name':    'map',
            'frame_id':      'map',
        }],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart':    True},
            {'node_names':   ['map_server']},
        ],
    )

    # ── Graph Visualization ───────────────────────────────────────────────────
    graph_json_file = os.path.join(
        robot_navigation_dir, 'graphs', 'robot_map_graph.json'
    )
    graph_visualizer_node = Node(
        package='robot_navigation',
        executable='visualize_graph_rviz.py',
        name='graph_visualizer',
        arguments=[graph_json_file],
        output='screen',
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(
        robot_navigation_dir, 'rviz', 'robot_rviz_config.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    ld.add_action(localization_sim_launch)  # sim_slamware_bridge + EKF + static TF map→odom
    ld.add_action(custom_navigation_launch)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_map)
    ld.add_action(graph_visualizer_node)
    ld.add_action(rviz_node)

    return ld
