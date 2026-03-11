"""
Localization Benchmark Launch
==============================
Starts the benchmark node that compares a localization algorithm
against the Gazebo ground truth.

Usage:
  # Compare FGO (default)
  ros2 launch robot_gazebo localization_benchmark.launch.py

  # Compare a different algorithm
  ros2 launch robot_gazebo localization_benchmark.launch.py \
      estimated_topic:=/amcl_pose_as_odom \
      algorithm_name:=AMCL

  # Custom CSV output directory
  ros2 launch robot_gazebo localization_benchmark.launch.py \
      csv_output_dir:=/home/user/results
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────
    declare_estimated_topic = DeclareLaunchArgument(
        "estimated_topic",
        default_value="/fgo/odometry",
        description="nav_msgs/Odometry topic published by the algorithm under test",
    )
    declare_algorithm_name = DeclareLaunchArgument(
        "algorithm_name",
        default_value="FGO",
        description="Name tag used in logs and CSV filename",
    )
    declare_csv_dir = DeclareLaunchArgument(
        "csv_output_dir",
        default_value="/tmp",
        description="Directory where the CSV benchmark report will be saved on shutdown",
    )
    declare_sync_slop = DeclareLaunchArgument(
        "sync_slop_sec",
        default_value="0.15",
        description="ApproximateTimeSynchronizer tolerance in seconds",
    )

    # ── Benchmark node ─────────────────────────────────────────────────────
    benchmark_node = Node(
        package="robot_gazebo",
        executable="localization_benchmark.py",
        name="localization_benchmark",
        output="screen",
        parameters=[
            {
                "use_sim_time":     True,
                "estimated_topic":  LaunchConfiguration("estimated_topic"),
                "algorithm_name":   LaunchConfiguration("algorithm_name"),
                "csv_output_dir":   LaunchConfiguration("csv_output_dir"),
                "sync_slop_sec":    LaunchConfiguration("sync_slop_sec"),
            }
        ],
    )

    return LaunchDescription([
        declare_estimated_topic,
        declare_algorithm_name,
        declare_csv_dir,
        declare_sync_slop,
        benchmark_node,
    ])
