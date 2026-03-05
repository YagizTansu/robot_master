from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Package directories
    robot_navigation_dir = get_package_share_directory("robot_navigation")
    robot_database_dir = get_package_share_directory("robot_database")
    fgo_dir = get_package_share_directory("factor_graph_optimization")

    # Map
    map_file = os.path.join(robot_navigation_dir, "map", "aws_warehouse.yaml")

    # Path to the robot map graph JSON file
    graph_json_file = os.path.join(
        robot_navigation_dir, "graphs", "robot_map_graph.json"
    )

    # ── FGO Localization (replaces AMCL + EKF) ────────────────────────────────
    # Provides:
    #   map → odom           (iSAM2 optimised result — replaces AMCL)
    #   odom → base_footprint (raw odom pass-through    — replaces EKF)
    fgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fgo_dir, "launch", "fgo.launch.py"))
    )

    # ── Nav2 Navigation Stack ─────────────────────────────────────────────────
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

    # ── Map Server ────────────────────────────────────────────────────────────
    # A standalone map server is required because FGO relies on it, and
    # the custom nav2 launch does not include it.
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,  # Hardcoded to true for now since launch configs are generally used for sim
                "yaml_filename": map_file,
                "topic_name": "map",
                "frame_id": "map",
            }
        ],
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
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

    ld.add_action(fgo_launch)
    ld.add_action(custom_navigation_launch)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_map)
    ld.add_action(graph_visualizer_node)
    ld.add_action(robot_database_launch)

    return ld
