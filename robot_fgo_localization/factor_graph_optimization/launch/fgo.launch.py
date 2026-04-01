import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Launch configurations ───────────────────────────────────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # ── Parameter file ─────────────────────────────────────────────────────
    params_file = os.path.join(
        get_package_share_directory("factor_graph_optimization"),
        "config",
        "fgo_params.yaml",
    )

    # ── fgo_node ───────────────────────────────────────────────────────────
    # Replaces AMCL (map→odom TF) and EKF (odom→base_footprint TF)
    fgo_node = Node(
        package="factor_graph_optimization",
        executable="fgo_node",
        name="fgo_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── scan_matcher_node ───────────────────────────────────────────────────────────
    # Subscribes: /map (transient_local), /scan
    # Publishes:  /scan_match_pose (PoseWithCovarianceStamped)
    scan_matcher_node = Node(
        package="factor_graph_optimization",
        executable="scan_matcher_node",
        name="scan_matcher_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # ── trust_weight_bridge node ───────────────────────────────────────────
    # Bridges /transformer/trust_scores → /fgo/trust_weights.
    # Runs in manual_mode=true by default (weights from YAML).
    # Switch to Transformer mode at runtime:
    #   ros2 service call /fgo/set_trust_weights std_srvs/srv/SetBool "{data: false}"
    trust_weight_bridge_node = Node(
        package="factor_graph_optimization",
        executable="trust_weight_bridge",
        name="trust_weight_bridge",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            trust_weight_bridge_node,
            fgo_node,
            scan_matcher_node,
        ]
    )
