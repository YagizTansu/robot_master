import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                             RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    robot_slam_dir = get_package_share_directory('robot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            robot_slam_dir, 'config', 'slam_toolbox_localization.yaml'
        ),
        description='Full path to the slam_toolbox localization parameters file',
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(robot_slam_dir, 'maps', 'warehouse_map'),
        description='Full path to the map file (without .yaml / .posegraph extension)',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate slam_toolbox lifecycle node.',
    )

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='false',
        description='Enable bond connection during node activation (Nav2 lifecycle manager)',
    )

    # localization_slam_toolbox_node is a LifecycleNode — must be configured+activated
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
                'use_lifecycle_manager': use_lifecycle_manager,
            },
        ],
    )

    # Trigger: unconfigured → inactive
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    # Trigger: inactive → active (once configure succeeds)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox localization node is activating.'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_map_file,
        declare_autostart,
        declare_use_lifecycle_manager,
        slam_node,
        configure_event,
        activate_event,
    ])
