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

    # Include robot_dynamic_warehouse.launch.py
    robot_warehouse_launch = IncludeLaunchDescription(
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
        robot_warehouse_launch
    ])
    
    # Simulation-only BT prerequisite topic publishers.
    # Publishes /emergency_stop, /battery_status, /diagnostics_agg,
    # and /fleet_manager/heartbeat so the idle-monitor BT can run in simulation.
    # Override parameters to inject fault scenarios, e.g.:
    #   ros2 param set /sim_bt_topic_publishers battery_percentage 15.0
    sim_bt_publishers_node = Node(
        package='robot_gazebo',
        executable='sim_bt_topic_publishers.py',
        name='sim_bt_topic_publishers',
        output='screen',
        parameters=[{
            'publish_rate_hz':          2.0,
            'emergency_active':         False,
            'battery_percentage':       85.0,
            'battery_voltage':          24.0,
            'fleet_online':             True,
            'diagnostics_ok':           True,
            'motor_temperature_celsius': 35.0,
        }],
    )
    ld.add_action(sim_bt_publishers_node)

    return ld
