Maps saved here by map_saver_cli.

Save command:
    ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg prefix robot_slam)/share/robot_slam/maps/<map_name>

Or save to home directory (simpler):
    ros2 run nav2_map_server map_saver_cli -f ~/maps/<map_name>
