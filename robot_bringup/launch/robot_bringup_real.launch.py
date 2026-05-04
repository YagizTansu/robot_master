from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    homing = LaunchConfiguration('homing', default='true')
    ld.add_action(DeclareLaunchArgument(
        'homing',
        default_value='true',
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

    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode', default='Sensitivity')
    ld.add_action(DeclareLaunchArgument(
        'lidar_scan_mode',
        default_value='Sensitivity',
        description='RPLidar S2E scan mode',
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

    # ── Slamware ROS SDK ──────────────────────────────────────────────────────
    # Publishes:
    #   /scan              → sensor_msgs/LaserScan  (remap → /lidar_top/scan)
    #   /map               → nav_msgs/OccupancyGrid
    #   /slamware_ros_sdk_server_node/robot_pose → geometry_msgs/PoseStamped
    slamware_node = Node(
        package='slamware_ros_sdk',
        executable='slamware_ros_sdk_server_node',
        name='slamware_ros_sdk_server_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'ip_address': slamware_ip,
            # Slamware'in yayınladığı frame → URDF'deki lidar frame ile eşleşmeli
            'robot_frame': 'base_footprint',
            'laser_frame': 'lidar_top',
            'map_frame': 'map',
            'odom_frame': 'odom',
            # Slamware kendi haritasını TF olarak da yayınlıyor;
            # AMCL kullandığımız için Slamware'in map→odom TF'ini kapatıyoruz
            'publish_tf': False,
        }],
        remappings=[
            # AMCL ve nav2 /lidar_top/scan topic'ini bekliyor
            ('/scan', '/lidar_top/scan'),
        ],
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
    ld.add_action(slamware_node)
    ld.add_action(rplidar_s2e_launch)

    return ld
