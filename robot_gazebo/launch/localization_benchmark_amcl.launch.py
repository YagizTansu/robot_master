"""
AMCL Localization Benchmark Launch
====================================
Starts the AMCL-pose converter and the benchmark node that compares
AMCL localization against the Gazebo ground truth.

Prerequisites (run in separate terminals / launch files):
  1. Gazebo simulation with ground_truth_publisher running
     → publishes /ground_truth/odom
  2. AMCL running (e.g. via robot_navigation_amcl_ekf.launch.py)
     → publishes /amcl_pose

Usage:
  ros2 launch robot_gazebo localization_benchmark_amcl.launch.py

  # Custom CSV output directory
  ros2 launch robot_gazebo localization_benchmark_amcl.launch.py \
      csv_output_dir:=/home/user/results
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────
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

    # ── AMCL pose → Odometry converter ────────────────────────────────────
    # AMCL publishes geometry_msgs/PoseWithCovarianceStamped on /amcl_pose.
    # The benchmark node expects nav_msgs/Odometry, so we bridge them here.
    amcl_to_odom_node = Node(
        package="robot_gazebo",
        executable="amcl_pose_to_odom.py",
        name="amcl_pose_to_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time":  True,
                "input_topic":   "/amcl_pose",
                "output_topic":  "/amcl_pose_as_odom",
            }
        ],
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
                "estimated_topic":  "/amcl_pose_as_odom",
                "algorithm_name":   "AMCL",
                "csv_output_dir":   LaunchConfiguration("csv_output_dir"),
                "sync_slop_sec":    LaunchConfiguration("sync_slop_sec"),
            }
        ],
    )

    return LaunchDescription([
        declare_csv_dir,
        declare_sync_slop,
        amcl_to_odom_node,
        benchmark_node,
    ])
