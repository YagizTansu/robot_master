"""
robot_slam — mapping.launch.py

Starts slam_toolbox in online_async mapping mode + FGO node for odom TF.

TF tree during mapping:
    map → odom          ← slam_toolbox (disabled in FGO via publish_map_to_odom=false)
    odom → base_footprint ← FGO node (odomCallback → publishOdomToBase)

Usage:
    ros2 launch robot_slam mapping.launch.py
    ros2 launch robot_slam mapping.launch.py use_sim_time:=false   # real robot

After mapping:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/<map_name>

Then switch to localization:
    ros2 launch robot_slam localization.launch.py map:=~/maps/<map_name>.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (Gazebo). Set false on real hardware.',
    )

    # ── Package paths ─────────────────────────────────────────────────────────
    robot_slam_dir      = get_package_share_directory('robot_slam')
    fgo_dir             = get_package_share_directory('factor_graph_optimization')
    robot_navigation_dir = get_package_share_directory('robot_navigation')

    slam_params_file = os.path.join(
        robot_slam_dir, 'config', 'slam_toolbox_mapping.yaml'
    )
    fgo_params_file = os.path.join(fgo_dir, 'config', 'fgo_params.yaml')
    rviz_config_file = os.path.join(
        robot_navigation_dir, 'rviz', 'robot_slam_config.rviz'
    )

    # ── FGO node — odom → base_footprint TF only ──────────────────────────────
    # publish_map_to_odom is OVERRIDDEN to false so FGO does not compete with
    # slam_toolbox for the map→odom TF.  FGO still runs its graph and publishes
    # odom→base_footprint via odomCallback → publishOdomToBase(), which
    # slam_toolbox requires to build the map.
    fgo_node = Node(
        package='factor_graph_optimization',
        executable='fgo_node',
        name='fgo_node',
        output='screen',
        parameters=[
            fgo_params_file,
            {
                'use_sim_time': use_sim_time,
                # Disable map→odom TF — slam_toolbox owns this during mapping.
                'tf.publish_map_to_odom': False,
            },
        ],
    )

    # ── slam_toolbox — online async mapping ───────────────────────────────────
    # Subscribes : /scan, /tf (odom→base_footprint supplied by fgo_node above)
    # Publishes  : /map (OccupancyGrid, transient_local)
    #              map→odom TF
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Lifecycle manager ─────────────────────────────────────────────────────
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox'],
        }],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        fgo_node,
        slam_toolbox_node,
        lifecycle_manager_node,
        rviz_node,
    ])
