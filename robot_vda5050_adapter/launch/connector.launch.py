import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch VDA5050 full stack: MQTT bridge + controller + robot adapter."""

    package_name = "robot_vda5050_adapter"
    package_dir = get_package_share_directory(package_name)

    connector_package = "vda5050_connector"
    connector_dir = get_package_share_directory(connector_package)

    namespace = LaunchConfiguration("namespace")
    parameters_config_file = LaunchConfiguration("parameters_config_file")

    # --- Declare arguments ---
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="vda5050_connector",
        description="Namespace for all VDA5050 nodes",
    )

    declare_config = DeclareLaunchArgument(
        "parameters_config_file",
        default_value=os.path.join(package_dir, "config", "connector.yaml"),
        description="Full path to the VDA5050 connector config file",
    )

    # --- Nodes ---
    mqtt_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(connector_dir, "launch", "mqtt_bridge.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "parameters_config_file": parameters_config_file,
        }.items(),
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(connector_dir, "launch", "controller.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "parameters_config_file": parameters_config_file,
        }.items(),
    )

    adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "adapter.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "parameters_config_file": parameters_config_file,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_config)
    ld.add_action(mqtt_bridge)
    ld.add_action(controller)
    ld.add_action(adapter)
    return ld
