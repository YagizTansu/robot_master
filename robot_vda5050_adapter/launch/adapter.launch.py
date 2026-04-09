import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the robot VDA5050 adapter node."""

    package_name = "robot_vda5050_adapter"
    package_dir = get_package_share_directory(package_name)

    namespace = LaunchConfiguration("namespace")
    parameters_config_file = LaunchConfiguration("parameters_config_file")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="vda5050_connector",
        description="Namespace for the adapter node",
    )

    declare_config = DeclareLaunchArgument(
        "parameters_config_file",
        default_value=os.path.join(package_dir, "config", "connector.yaml"),
        description="Full path to the VDA5050 connector config file",
    )

    adapter_node = Node(
        package=package_name,
        executable="robot_adapter",
        name="robot_adapter",
        namespace=namespace,
        parameters=[parameters_config_file],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_config)
    ld.add_action(adapter_node)
    return ld
