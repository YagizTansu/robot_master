import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the path to robot_params.yaml
    robot_bringup_pkg = get_package_share_directory('robot_bringup')
    robot_params_file = os.path.join(robot_bringup_pkg, 'robot_startup_param', 'robot_params.yaml')
    
    # Load robot_bringup parameters to extract robot_name
    import yaml
    with open(robot_params_file, 'r') as f:
        params = yaml.safe_load(f)
    
    robot_name = params.get('robot_bringup', {}).get('ros__parameters', {}).get('robot_name', 'default_robot')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Robot database node
    robot_database_node = Node(
        package='robot_database',
        executable='robot_database_node',
        name='robot_database_node',
        parameters=[
            {
                'robot_name': robot_name,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )
    
    # Robot database stream node
    robot_database_stream_node = Node(
        package='robot_database',
        executable='robot_database_stream_node',
        name='robot_database_stream_node',
        parameters=[
            {
                'robot_name': robot_name,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_database_node,
        robot_database_stream_node
    ])