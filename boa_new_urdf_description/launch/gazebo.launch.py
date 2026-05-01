from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('boa_new_urdf_description')
    robot_gazebo_dir = get_package_share_directory('robot_gazebo')

    xacro_file = os.path.join(share_dir, 'urdf', 'boa_new_urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    default_world = os.path.join(share_dir, 'worlds', 'robot_world.sdf')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to world SDF file'
    )

    world = LaunchConfiguration('world')

    xacro_file = os.path.join(share_dir, 'urdf', 'boa_new_urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # ── Robot state publisher ────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf, 'use_sim_time': True}]
    )

    # ── Gazebo simulator ─────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world]
        }.items()
    )

    # ── Spawn robot into Gazebo ──────────────────────────────────────────────
    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'boa_new_urdf', '-topic', 'robot_description', '-z', '0.15'],
        output='screen'
    )

    # ── Gazebo ↔ ROS2 bridges (clock + lidar) ───────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': os.path.join(share_dir, 'config', 'ros_gz_bridge_gazebo.yaml'),
            'use_sim_time': True,
        }],
        output='screen'
    )

    # ── Controllers (spawned after Gazebo + gz_ros2_control are ready) ───────
    controllers_yaml = os.path.join(share_dir, 'config', 'controllers.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_controller', '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    traction_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['traction_controller', '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    lift_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['lift_controller', '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Kinco sürüş modu: cmd_vel → steering + traction controller ───────────
    kinco_drive_node = Node(
        package='robot_motor_controller',
        executable='kinco_drive_node',
        name='kinco_drive_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Simülasyon odometrisi: joint_states → /odom_kinco ────────────────────
    sim_kinco_bridge = Node(
        package='robot_motor_controller',
        executable='sim_kinco_bridge',
        name='sim_kinco_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        # Make warehouse models visible to Gazebo
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.join(robot_gazebo_dir, 'models')),
        robot_state_publisher_node,
        gazebo,
        urdf_spawn_node,
        gz_bridge,
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=6.0, actions=[steering_controller_spawner]),
        TimerAction(period=6.0, actions=[traction_controller_spawner]),
        TimerAction(period=7.0, actions=[lift_controller_spawner]),
        TimerAction(period=8.0, actions=[kinco_drive_node]),
        TimerAction(period=8.0, actions=[sim_kinco_bridge]),
    ])


