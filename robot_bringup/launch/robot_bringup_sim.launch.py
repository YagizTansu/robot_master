from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')

    lab_new_world = os.path.join(
        robot_gazebo_dir, 'worlds', 'lab_new.sdf'
    )

    robot_gazebo_models = os.path.join(robot_gazebo_dir, 'models')

    # Include gazebo.launch.py with the lab_new world
    robot_warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': lab_new_world}.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', robot_gazebo_models),
        robot_warehouse_launch,
    ])

