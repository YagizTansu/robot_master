"""
robot_slam — localization.launch.py

Serves a pre-built map and starts the FGO localization stack.

Usage:
    ros2 launch robot_slam localization.launch.py map:=/full/path/to/map.yaml
    ros2 launch robot_slam localization.launch.py map:=/full/path/to/map.yaml use_sim_time:=false

Pipeline:
    map_server  →  /map (transient_local OccupancyGrid)
        ↓
    scan_matcher_node  →  /scan_match_pose
        ↓
    fgo_node  →  /fgo/odometry  +  map→odom TF
        ↓
    Nav2

NOTE: slam_toolbox is NOT started here. FGO publishes map→odom TF.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (Gazebo). Set false on real hardware.',
    )
    declare_map = DeclareLaunchArgument(
        'map',
        description='Full path to the map YAML file produced by map_saver_cli.',
    )

    # ── Package paths ─────────────────────────────────────────────────────────
    fgo_dir = get_package_share_directory('factor_graph_optimization')
    fgo_params_file = os.path.join(fgo_dir, 'config', 'fgo_params.yaml')
    fgo_launch_file = os.path.join(fgo_dir, 'launch', 'fgo.launch.py')

    # ── map_server — serves the saved OccupancyGrid ───────────────────────────
    # QoS: transient_local so scan_matcher_node receives the map even if it
    # started after map_server (late-join subscriber gets the latched message).
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }],
    )

    # ── Lifecycle manager for map_server ──────────────────────────────────────
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': ['map_server'],
        }],
    )

    # ── FGO localization stack ────────────────────────────────────────────────
    # Starts: fgo_node + scan_matcher_node + trust_weight_bridge
    # (defined in factor_graph_optimization/launch/fgo.launch.py)
    fgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fgo_launch_file),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        map_server_node,
        lifecycle_manager_map,
        fgo_launch,
    ])
