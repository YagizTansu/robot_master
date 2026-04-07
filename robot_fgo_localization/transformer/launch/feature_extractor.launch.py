"""
Feature Extractor Launch
=========================
Starts the feature_extractor_node that records sensor features + ground truth
error labels to CSV for offline Transformer training.

Prerequisites (must already be running):
  - Gazebo simulation (robot_warehouse.launch.py)
  - FGO localization stack  (fgo_node + scan_matcher_node + trust_weight_bridge)
  - Ground truth publisher  (ground_truth_publisher node)

Usage:
  ros2 launch fgo_transformer feature_extractor.launch.py

  # Custom output directory:
  ros2 launch fgo_transformer feature_extractor.launch.py \\
      csv_output_dir:=~/my_training_data
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:

    # ── Launch arguments ───────────────────────────────────────────────────
    declare_csv_dir = DeclareLaunchArgument(
        "csv_output_dir",
        default_value=os.path.expanduser("~/fgo_training_data"),
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

    # ── Feature extractor node ─────────────────────────────────────────────
    feature_extractor_node = Node(
        package="fgo_transformer",
        executable="feature_extractor",
        name="feature_extractor",
        output="screen",
        parameters=[
            # Load YAML defaults first, then allow launch-arg overrides
            os.path.join(
                os.path.dirname(__file__),
                "..", "config", "feature_extractor_params.yaml",
            ),
            {
                "use_sim_time":    True,
                "csv_output_dir":  LaunchConfiguration("csv_output_dir"),
                "sync_slop_sec":   LaunchConfiguration("sync_slop_sec"),
                "imu_buffer_size": LaunchConfiguration("imu_buffer_size"),
            },
        ],
    )

    return LaunchDescription([
        declare_csv_dir,
        declare_sync_slop,
        declare_imu_buf,
        feature_extractor_node,
    ])
