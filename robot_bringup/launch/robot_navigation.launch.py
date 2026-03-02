from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get the robot_navigation package directory
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_database_dir = get_package_share_directory('robot_database')

    # Path to the robot map graph JSON file
    graph_json_file = os.path.join(robot_navigation_dir, 'graphs', 'robot_map_graph.json')
    
    # Include robot_amcl_navigation2.launch.py
    robot_amcl_navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_dir, 'launch', 'robot_amcl_navigation2.launch.py')
        )
    )
    
    # Graph visualization node
    graph_visualizer_node = Node(
        package='robot_navigation',
        executable='visualize_graph_rviz.py',
        name='graph_visualizer',
        arguments=[graph_json_file],
        output='screen'
    )
    
    # Include robot_database.launch.py
    robot_database_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_database_dir, 'launch', 'robot_database.launch.py')
        )
    )
    
    ld.add_action(robot_amcl_navigation2_launch)
    ld.add_action(graph_visualizer_node)
    ld.add_action(robot_database_launch)
   
    return ld
