from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import xacro
import os


def generate_launch_description():
    ld = LaunchDescription()

    # ── Launch Arguments ──────────────────────────────────────────────────────
    slamware_ip = LaunchConfiguration('slamware_ip', default='192.168.11.1')
    ld.add_action(DeclareLaunchArgument(
        'slamware_ip',
        default_value='192.168.11.1',
        description='Slamware mapper IP address',
    ))

    slamware_new_map = LaunchConfiguration('slamware_new_map', default='false')
    ld.add_action(DeclareLaunchArgument(
        'slamware_new_map',
        default_value='false',
        description='Whether upload map or not',
    ))

    homing = LaunchConfiguration('homing')
    ld.add_action(DeclareLaunchArgument(
        'homing',
        default_value='false',
        description='Run steering homing procedure on startup',
    ))

    lidar_udp_ip = LaunchConfiguration('lidar_udp_ip', default='192.168.11.2')
    ld.add_action(DeclareLaunchArgument(
        'lidar_udp_ip',
        default_value='192.168.11.2',
        description='RPLidar S2E UDP IP address',
    ))

    lidar_udp_port = LaunchConfiguration('lidar_udp_port', default='8089')
    ld.add_action(DeclareLaunchArgument(
        'lidar_udp_port',
        default_value='8089',
        description='RPLidar S2E UDP port',
    ))

    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='lidar_fork_1')
    ld.add_action(DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='lidar_fork_1',
        description='RPLidar S2E frame_id',
    ))

    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode', default='DenseBoost')
    ld.add_action(DeclareLaunchArgument(
        'lidar_scan_mode',
        default_value='DenseBoost',
        description='RPLidar S2E scan mode'
    ))

    lidar_scan_frequency = LaunchConfiguration('lidar_scan_frequency', default='10')
    ld.add_action(DeclareLaunchArgument(
        'lidar_scan_frequency',
        default_value='10',
        description='RPLidar S2E scan frequency (Hz)',
    ))

    # ── Robot State Publisher (URDF static TFs) ──────────────────────────────
    # base_footprint → base_link, base_link → lidar_top, vb.
    boa_dir = get_package_share_directory('boa_new_urdf_description')
    xacro_file = os.path.join(boa_dir, 'urdf', 'boa_new_urdf.xacro')
    robot_urdf = xacro.process_file(xacro_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_urdf,
        }],
    )

    # ── Kinco Bridge (motor controller + raw odometry) ────────────────────────
    # Publishes:  /odom_kinco → nav_msgs/Odometry
    # Subscribes: /cmd_vel   → geometry_msgs/Twist
    kinco_bridge_node = Node(
        package='robot_motor_controller',
        executable='kinco_bridge',
        name='kinco_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'homing': homing,
        }],
    )


    # ── Slamware Aurora SDK ─────────────────────────────────────────────────
    slamware_ros_sdk_dir = get_package_share_directory('slamware_ros_sdk')

    slamware_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(slamware_ros_sdk_dir, 'launch', 'slamware_ros_sdk_server_and_view.xml')
        ),
        launch_arguments={
            'ip_address': slamware_ip,
            'slamware_new_map': slamware_new_map
        }.items(),
    )

    # ── Scan Relay (slamware scan → /lidar_top/scan) ─────────────────────────
    scan_relay_node = Node(
        package='slamware_ros_sdk',
        executable='scan_relay_node',
        name='scan_relay',
        output='screen',
    )

    # ── RPLidar S2E ───────────────────────────────────────────────────────────
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')

    rplidar_s2e_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros_dir, 'launch', 'rplidar_s2e_launch.py')
        ),
        launch_arguments={
            'udp_ip': lidar_udp_ip,
            'udp_port': lidar_udp_port,
            'frame_id': lidar_frame_id,
            'scan_mode': lidar_scan_mode,
            'scan_frequency': lidar_scan_frequency,
        }.items(),
    )


    ld.add_action(robot_state_publisher_node)
    ld.add_action(kinco_bridge_node)
    ld.add_action(slamware_launch)
    ld.add_action(scan_relay_node)
    ld.add_action(rplidar_s2e_launch)

    return ld
