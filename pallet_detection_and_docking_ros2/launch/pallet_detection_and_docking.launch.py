import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('pallet_detection_and_docking')

    cluster_params = PathJoinSubstitution([pkg_share, 'params', 'cluster_params.yaml'])

    return LaunchDescription([
        # ── Launch Arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error, fatal)'
        ),

        # ── pallet_detection ─────────────────────────────────────────────────
        Node(
            package='pallet_detection_and_docking',
            executable='pallet_detection',
            name='pallet_detection',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[cluster_params],
        ),

        # ── pallet_docking ────────────────────────────────────────────────────
        Node(
            package='pallet_detection_and_docking',
            executable='pallet_docking',
            name='pallet_docking',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),

        # path_follower burada başlatılmıyor.
        # Docking hareketi nav2 controller_server (DockingPath profili) üzerinden
        # FollowPath action ile yapılıyor. path_follower aynı cmd_vel'e yazarak
        # çakışma yaratırdı.
    ])
