from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    ld = LaunchDescription()

    # Get the package directories
    robot_gazebo_dir = get_package_share_directory("robot_gazebo")
    robot_description_dir = get_package_share_directory("robot_description")
    dual_laser_merger_dir = get_package_share_directory("dual_laser_merger")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # URDF
    xacro_file = os.path.join(robot_description_dir, "urdf", "robot.urdf.xacro")
    assert os.path.exists(xacro_file), "URDF file doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # Include robot_warehouse.launch.py
    robot_warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_dir, "launch", "robot_warehouse.launch.py")
        )
    )

    # Include demo_laser_merger.launch.py
    # Merges dual lidar scans → /scan topic consumed by FGO node
    demo_laser_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_laser_merger_dir, "launch", "demo_laser_merger.launch.py")
        )
    )

    ld.add_action(robot_warehouse_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(demo_laser_merger_launch)

    return ld
