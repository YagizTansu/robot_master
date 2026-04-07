#!/usr/bin/env python3
"""Launch the Transformer inference node.

Usage:
    ros2 launch fgo_transformer transformer_inference.launch.py

Optional override:
    ros2 launch fgo_transformer transformer_inference.launch.py \
        checkpoint_path:=/home/yagiz/fgo_training_data/checkpoints/best.pt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("fgo_transformer")
    default_ckpt = os.path.join(
        os.path.expanduser("~"), "fgo_training_data", "checkpoints", "best.pt"
    )

    ckpt_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value=default_ckpt,
        description="Absolute path to the trained best.pt checkpoint",
    )

    inference_node = Node(
        package="fgo_transformer",
        executable="transformer_inference",
        name="transformer_inference",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(pkg_share, "config", "transformer_inference_params.yaml"),
            {"checkpoint_path": LaunchConfiguration("checkpoint_path")},
        ],
    )

    return LaunchDescription([ckpt_arg, inference_node])
