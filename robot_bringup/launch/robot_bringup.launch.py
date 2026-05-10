from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    boa_dir = get_package_share_directory('boa_new_urdf_description')
    map2gazebo_dir = get_package_share_directory('map2gazebo')

    lab_new_world = os.path.join(
        map2gazebo_dir, 'worlds', 'lab_new.sdf'
    )

    # Make lab_new model (with the STL mesh) visible to Gazebo
    map2gazebo_models = os.path.join(map2gazebo_dir, 'models')

    # Include gazebo.launch.py with the lab_new world
    robot_warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boa_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': lab_new_world}.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', map2gazebo_models),
        robot_warehouse_launch,
    ])

