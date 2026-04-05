import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    map_save_dir = LaunchConfiguration('map_save_dir')

    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name of the map file to save (without extension)',
    )

    declare_map_save_dir = DeclareLaunchArgument(
        'map_save_dir',
        default_value=os.path.join(
            os.path.expanduser('~'), 'ros2_ws', 'src', 'robot_master',
            'robot_slam', 'maps'
        ),
        description='Directory where the map will be saved',
    )

    # use_sim_time is intentionally false here:
    # map_saver_cli only needs to read one /map message; giving it sim time
    # causes it to time-out waiting for a /clock tick before the subscription
    # can be served.
    save_map = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', PathJoinSubstitution([map_save_dir, map_name]),
            '--ros-args', '-p', 'use_sim_time:=false',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_map_name,
        declare_map_save_dir,
        save_map,
    ])
