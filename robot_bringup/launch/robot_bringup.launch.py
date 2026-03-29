from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    boa_dir = get_package_share_directory('boa_new_urdf_description')
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')
    dual_laser_merger_dir = get_package_share_directory('dual_laser_merger')

    warehouse_world = os.path.join(
        robot_gazebo_dir, 'worlds', 'robot_warehouse.sdf'
    )

    # Gazebo + robot spawn + bridge + controllers (boa_new_urdf)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(boa_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': warehouse_world}.items()
    )

    # Merges dual lidar scans → /scan topic consumed by FGO node
    demo_laser_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dual_laser_merger_dir, 'launch', 'demo_laser_merger.launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch,
        demo_laser_merger_launch,
    ])

