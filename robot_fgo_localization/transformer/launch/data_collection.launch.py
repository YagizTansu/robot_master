"""
Data Collection Launch
======================
Starts feature_extractor + a driver node that automatically navigates the
robot to collect training data.  Two modes are available:

  mode:=graph_traversal  (default)
      Drives the robot through EVERY edge in robot_map_graph.json at least
      once, providing complete spatial coverage of the environment.

  mode:=scenario
      Runs a predefined set of kinematic stress scenarios (jerk bursts,
      spins, slaloms, etc.) that push individual feature dimensions to
      their extremes for better label diversity.

  mode:=both
      Runs graph_traversal first, then scenario runner **sequentially**.
      scenario_runner waits for graph_traversal's COMPLETE signal before
      starting.  Both nodes shut down automatically when all scenarios finish.

Prerequisites (must already be running before this launch):
  - Gazebo simulation         (robot_warehouse.launch.py)
  - FGO localization stack    (fgo_node, scan_matcher_node, trust_weight_bridge)
  - Ground truth publisher    (ground_truth_publisher node)
  - Nav2 stack                (navigation2_launch.py / custom_navigation_launch.py)

Usage
-----
  # Full coverage traversal (recommended first run):
  ros2 launch fgo_transformer data_collection.launch.py

  # Scenario-only run:
  ros2 launch fgo_transformer data_collection.launch.py mode:=scenario

  # Both modes sequentially — graph traversal finishes, then scenarios start:
  ros2 launch fgo_transformer data_collection.launch.py mode:=both

  # Custom output directory:
  ros2 launch fgo_transformer data_collection.launch.py \\
      csv_output_dir:=~/training_data

  # Custom starting node:
  ros2 launch fgo_transformer data_collection.launch.py \\
      start_node:=node_0_0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    _pkg_share = get_package_share_directory("fgo_transformer")

    # ── Find graph file ────────────────────────────────────────────────────
    # Walk up from the package share to locate robot_map_graph.json.
    # Adjust this path if the workspace layout changes.
    _ws_src = os.path.normpath(
        os.path.join(_pkg_share, "..", "..", "..", "..", "src")
    )
    _default_graph = os.path.join(
        _ws_src,
        "robot_master", "robot_navigation", "graphs", "robot_map_graph.json",
    )

    # ── Launch arguments ───────────────────────────────────────────────────
    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="graph_traversal",
        description=(
            "Collection driver mode: "
            "'graph_traversal' | 'scenario' | 'both'"
        ),
    )
    declare_csv_dir = DeclareLaunchArgument(
        "csv_output_dir",
        default_value=os.path.join(_pkg_share, "data"),
        description="Directory where CSV training files will be saved",
    )
    declare_sync_slop = DeclareLaunchArgument(
        "sync_slop_sec",
        default_value="0.15",
        description="ApproximateTimeSynchronizer tolerance (seconds)",
    )
    declare_imu_buf = DeclareLaunchArgument(
        "imu_buffer_size",
        default_value="50",
        description="Number of IMU samples kept for std-dev computation",
    )
    declare_graph = DeclareLaunchArgument(
        "graph_file",
        default_value=_default_graph,
        description="Absolute path to robot_map_graph.json",
    )
    declare_start_node = DeclareLaunchArgument(
        "start_node",
        default_value="node_0_0",
        description="Graph node ID from which to begin traversal",
    )
    declare_nav_timeout = DeclareLaunchArgument(
        "nav_timeout_sec",
        default_value="120.0",
        description="Max seconds allowed per navigation goal",
    )
    declare_max_retries = DeclareLaunchArgument(
        "max_retries",
        default_value="3",
        description="Navigation retries before skipping a step",
    )
    declare_reset_node = DeclareLaunchArgument(
        "reset_node_id",
        default_value="node_0_0",
        description="Node ID the robot returns to between scenario runs",
    )
    declare_cycles = DeclareLaunchArgument(
        "repeat_cycles",
        default_value="3",
        description="How many full scenario cycles to run",
    )
    declare_cmd_vel = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="cmd_vel topic for scenario runner",
    )
    declare_wait_for_traversal = DeclareLaunchArgument(
        "wait_for_traversal",
        default_value="false",
        description=(
            "If true, scenario_runner waits for graph_traversal COMPLETE "
            "before starting (set automatically for mode:=both)"
        ),
    )

    # ── Feature extractor (always running) ────────────────────────────────
    _params_yaml = os.path.join(_pkg_share, "config", "feature_extractor_params.yaml")

    feature_extractor = Node(
        package="fgo_transformer",
        executable="feature_extractor",
        name="feature_extractor",
        output="screen",
        parameters=[
            _params_yaml,
            {
                "use_sim_time":    True,
                "csv_output_dir":  LaunchConfiguration("csv_output_dir"),
                "sync_slop_sec":   LaunchConfiguration("sync_slop_sec"),
                "imu_buffer_size": LaunchConfiguration("imu_buffer_size"),
            },
        ],
    )

    # ── Graph traversal node ───────────────────────────────────────────────
    _graph_traversal_params = {
        "use_sim_time":    True,
        "graph_file":      LaunchConfiguration("graph_file"),
        "start_node":      LaunchConfiguration("start_node"),
        "nav_timeout_sec": LaunchConfiguration("nav_timeout_sec"),
        "max_retries":     LaunchConfiguration("max_retries"),
    }

    graph_traversal = Node(
        package="fgo_transformer",
        executable="graph_traversal",
        name="graph_traversal",
        output="screen",
        parameters=[_graph_traversal_params],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "graph_traversal")),
    )

    graph_traversal_both = Node(
        package="fgo_transformer",
        executable="graph_traversal",
        name="graph_traversal",
        output="screen",
        parameters=[_graph_traversal_params],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "both")),
    )

    # ── Scenario runner node ───────────────────────────────────────────────
    _scenario_params = {
        "use_sim_time":   True,
        "graph_file":     LaunchConfiguration("graph_file"),
        "reset_node_id":  LaunchConfiguration("reset_node_id"),
        "repeat_cycles":  LaunchConfiguration("repeat_cycles"),
        "cmd_vel_topic":  LaunchConfiguration("cmd_vel_topic"),
    }

    scenario_runner = Node(
        package="fgo_transformer",
        executable="scenario_runner",
        name="scenario_runner",
        output="screen",
        parameters=[_scenario_params],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "scenario")),
    )

    scenario_runner_both = Node(
        package="fgo_transformer",
        executable="scenario_runner",
        name="scenario_runner",
        output="screen",
        parameters=[{
            **_scenario_params,
            "wait_for_traversal": True,  # wait for graph_traversal COMPLETE
        }],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("mode"), "both")),
    )

    return LaunchDescription([
        # Arguments
        declare_mode,
        declare_csv_dir,
        declare_sync_slop,
        declare_imu_buf,
        declare_graph,
        declare_start_node,
        declare_nav_timeout,
        declare_max_retries,
        declare_reset_node,
        declare_cycles,
        declare_cmd_vel,
        declare_wait_for_traversal,
        # Nodes
        feature_extractor,
        graph_traversal,
        graph_traversal_both,
        scenario_runner,
        scenario_runner_both,
    ])
