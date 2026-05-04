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

    homing = LaunchConfiguration('homing', default='false')
    ld.add_action(DeclareLaunchArgument(
        'homing',
        default_value='false',
        description='Run steering homing procedure on startup',
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

    # ── Navigation Stack ──────────────────────────────────────────────────────
    robot_bringup_dir = get_package_share_directory('robot_bringup')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_bringup_dir, 'launch', 'robot_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(kinco_bridge_node)
    ld.add_action(slamware_node)
    ld.add_action(navigation_launch)

    return ld
