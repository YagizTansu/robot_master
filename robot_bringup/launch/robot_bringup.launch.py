from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    boa_dir = get_package_share_directory('boa_new_urdf_description')
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')

    warehouse_world = os.path.join(
        robot_gazebo_dir, 'worlds', 'robot_warehouse.sdf'
    )

    # Include robot_dynamic_warehouse.launch.py
    robot_warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boa_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': warehouse_world}.items()
    )


    return LaunchDescription([
        robot_warehouse_launch
    ])

