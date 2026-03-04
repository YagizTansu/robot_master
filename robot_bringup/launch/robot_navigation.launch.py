from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Get the robot_navigation package directory
    robot_navigation_dir = get_package_share_directory("robot_navigation")
    robot_database_dir = get_package_share_directory("robot_database")
    robot_fgo_dir = get_package_share_directory("robot_fgo_localization")

    # Path to the robot map graph JSON file
    graph_json_file = os.path.join(
        robot_navigation_dir, "graphs", "robot_map_graph.json"
    )

    # ── FGO Localization (replaces AMCL + EKF) ────────────────────────────────
    # Launches: map_server, fgo_localization_node, pose saver/publisher
    robot_fgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_fgo_dir, "launch", "robot_fgo.launch.py")
        )
    )

    # ── Nav2 Navigation Stack ─────────────────────────────────────────────────
    # (previously included by robot_amcl_navigation2.launch.py via custom_navigation_launch.py)
    nav2_params_file = os.path.join(
        robot_navigation_dir, "config", "robot_move_base.yaml"
    )

    custom_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, "launch", "custom_navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": nav2_params_file,  # GraphBasedPlanner + CustomLocalPlanner
        }.items(),
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz_config_file = os.path.join(
        robot_navigation_dir, "rviz", "robot_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ── Graph Visualization ───────────────────────────────────────────────────
    graph_visualizer_node = Node(
        package="robot_navigation",
        executable="visualize_graph_rviz.py",
        name="graph_visualizer",
        arguments=[graph_json_file],
        output="screen",
    )

    # ── Database ──────────────────────────────────────────────────────────────
    robot_database_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_database_dir, "launch", "robot_database.launch.py")
        )
    )

    ld.add_action(robot_fgo_launch)
    ld.add_action(custom_navigation_launch)
    ld.add_action(rviz_node)
    ld.add_action(graph_visualizer_node)
    ld.add_action(robot_database_launch)

    return ld
