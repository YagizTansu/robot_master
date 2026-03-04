import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration("autostart", default="true")

    robot_fgo_dir = get_package_share_directory("robot_fgo_localization")
    robot_navigation_dir = get_package_share_directory("robot_navigation")

    # Params
    fgo_params_file = os.path.join(robot_fgo_dir, "params", "fgo_params.yaml")

    # Map — reuse existing aws_warehouse map
    map_file = os.path.join(robot_navigation_dir, "map", "aws_warehouse.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use Gazebo sim clock"
            ),
            DeclareLaunchArgument("autostart", default_value="true"),
            # ── Map Server ────────────────────────────────────────────────────────
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "yaml_filename": map_file,
                        "topic_name": "map",
                        "frame_id": "map",
                    }
                ],
            ),
            # Lifecycle manager for map_server only
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "bond_timeout": 4.0,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            Node(
                package="robot_navigation",
                executable="save_amcl_pose.py",
                name="fgo_pose_saver",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("/amcl_pose", "/fgo_pose")],
            ),
            Node(
                package="robot_navigation",
                executable="publish_initial_pose.py",
                name="initial_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "publish_delay_sec": 3.0,  # FGO node'un haritayı alması için bekle
                        "cov_x": 0.25,
                        "cov_y": 0.25,
                        "cov_yaw": 0.06854,
                    }
                ],
            ),
            # 3 seconds after start: publish last saved pose as /initialpose
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="robot_navigation",
                        executable="publish_initial_pose.py",
                        name="initial_pose_publisher",
                        output="screen",
                    ),
                ],
            ),
        ]
    )
