import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Package directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_database_dir   = get_package_share_directory('robot_database')
    robot_ekf_loc_dir    = get_package_share_directory('robot_ekf_localization')

    try:
        robot_slam_dir = get_package_share_directory('robot_slam')
    except PackageNotFoundError:
        robot_slam_dir = None

    try:
        vda5050_adapter_dir = get_package_share_directory('robot_vda5050_adapter')
    except PackageNotFoundError:
        vda5050_adapter_dir = None

    # Launch argument: enable/disable VDA5050 adapter
    enable_vda5050 = LaunchConfiguration("enable_vda5050", default="true")
    ld.add_action(DeclareLaunchArgument(
        "enable_vda5050",
        default_value="true",
        description="Launch VDA5050 adapter stack (MQTT bridge + controller + adapter)",
    ))

    # Map
    if robot_slam_dir and os.path.exists(os.path.join(robot_slam_dir, 'maps', 'warehouse_map.yaml')):
        map_file = os.path.join(robot_slam_dir, 'maps', 'warehouse_map.yaml')
    else:
        map_file = os.path.join(robot_navigation_dir, 'map', 'aws_warehouse.yaml')

    # Path to the robot map graph JSON file
    graph_json_file = os.path.join(
        robot_navigation_dir, "graphs", "robot_map_graph.json"
    )
    
    #ekf

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

    # ── VDA5050 Adapter Stack ─────────────────────────────────────────────────
    if vda5050_adapter_dir:
        vda5050_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vda5050_adapter_dir, 'launch', 'connector.launch.py')
            ),
            condition=launch.conditions.IfCondition(enable_vda5050),
        )
    else:
        vda5050_launch = None

    # ── EKF Localization (EKF local + EKF global + AMCL) ─────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_ekf_loc_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    ld.add_action(localization_launch)
    ld.add_action(custom_navigation_launch)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_map)
    ld.add_action(graph_visualizer_node)
    ld.add_action(robot_database_launch)
    if vda5050_launch:
        ld.add_action(vda5050_launch)

    return ld
