import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration("autostart", default="true")

    robot_fgo_dir = get_package_share_directory("robot_fgo_localization")
    robot_navigation_dir = get_package_share_directory("robot_navigation")

    fgo_params_file = os.path.join(robot_fgo_dir, "params", "fgo_params.yaml")
    map_file = os.path.join(robot_navigation_dir, "map", "aws_warehouse.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo sim clock",
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
            # Lifecycle manager — yalnızca map_server için
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
            # ── FGO Localization Node ─────────────────────────────────────────────
            Node(
                package="robot_fgo_localization",
                executable="fgo_localization_node",
                name="fgo_localization_node",
                output="screen",
                parameters=[
                    fgo_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            # ── Pose Persistence: Kaydet ──────────────────────────────────────────
            # /fgo_pose topic'ini dinler, 5sn'de bir ~/.ros/last_fgo_pose.json'a
            # atomik olarak yazar (eski save_amcl_pose.py'nin düzeltilmiş hali).
            Node(
                package="robot_navigation",
                executable="save_amcl_pose.py",
                name="fgo_pose_saver",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("/amcl_pose", "/fgo_pose")],
            ),
            # ── Pose Persistence: Yükle ───────────────────────────────────────────
            # Node başladıktan publish_delay_sec sonra (varsayılan: 2s) kayıtlı
            # pose'u /initialpose olarak yayınlar ve kapanır.
            # Not: TimerAction kaldırıldı — bekleme artık ROS2 timer ile node içinde
            # yönetiliyor (use_sim_time'a uyumlu).
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
        ]
    )
