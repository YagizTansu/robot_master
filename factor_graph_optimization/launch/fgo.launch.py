import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
        parameters=[params_file],
    )

    # ── scan_matcher_node ──────────────────────────────────────────────────
    # Subscribes: /map (transient_local), /scan
    # Publishes:  /scan_match_pose (PoseWithCovarianceStamped)
    scan_matcher_node = Node(
        package="factor_graph_optimization",
        executable="scan_matcher_node",
        name="scan_matcher_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            fgo_node,
            scan_matcher_node,
        ]
    )
