from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('boa_new_urdf_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'boa_new_urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf,
             'use_sim_time': True}
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # Bridge /clock from Gazebo → ROS2 so all nodes receive sim time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'boa_new_urdf',
            '-topic', 'robot_description',
            '-z', '0.15'
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file',
                   os.path.join(share_dir, 'config', 'controllers.yaml')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    tricycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tricycle_controller', '--param-file',
                   os.path.join(share_dir, 'config', 'controllers.yaml')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Convert /cmd_vel (Twist) → /tricycle_controller/cmd_vel (TwistStamped)
    # so nav2 / teleop can publish to standard /cmd_vel
    cmd_vel_relay = Node(
        package='boa_new_urdf_description',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        clock_bridge,
        urdf_spawn_node,
        cmd_vel_relay,
        # Wait for Gazebo + gz_ros2_control to start controller_manager
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=6.0, actions=[tricycle_controller_spawner]),
    ])


