import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Get the launch directory
    robot_gazebo_dir = get_package_share_directory("robot_gazebo")

    # Launch configuration variables specific to simulation

    world = LaunchConfiguration("world")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(robot_gazebo_dir, "worlds", "robot_warehouse.sdf"),
        description="Full path to world model file to load",
    )

    model_path = os.path.join(robot_gazebo_dir, "models")

    gazebo_server_cmd_line = ["gz", "sim", "-r", "-v4", world]

    gazebo = ExecuteProcess(cmd=gazebo_server_cmd_line, output="screen")

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/sick_lidar0/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/sick_lidar1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/kinect/depth@sensor_msgs/msg/Image@gz.msgs.Image",
            "/kinect/color@sensor_msgs/msg/Image@gz.msgs.Image",
            "/kinect/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/world/default/model/robot/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        ],
        remappings=[
            ("/model/robot/cmd_vel", "/cmd_vel"),
            ("/world/default/model/robot/joint_state", "/joint_state"),
            ("/world/default/clock", "clock"),
        ],
        output="screen",
    )

    # Ground truth publisher node
    # Reads /gz/dynamic_pose (TFMessage from Gz bridge), extracts the robot
    # model pose and republishes as:
    #   /ground_truth/odom  (nav_msgs/Odometry)
    #   /ground_truth/pose  (geometry_msgs/PoseStamped)
    ground_truth_publisher_node = Node(
        package="robot_gazebo",
        executable="ground_truth_publisher",
        name="ground_truth_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path))
    # Add any conditioned actions
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(ground_truth_publisher_node)

    return ld
