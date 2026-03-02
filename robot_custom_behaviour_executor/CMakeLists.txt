cmake_minimum_required(VERSION 3.10)
project(robot_custom_behaviour_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../robot_custom_behaviour_tree_nodes/include
)

# Monitor BT Node Executable
add_executable(monitor_bt_node
  src/monitor_bt_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../robot_custom_behaviour_tree_nodes/src/battery_check_action.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../robot_custom_behaviour_tree_nodes/src/emergency_stop_action.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../robot_custom_behaviour_tree_nodes/src/get_charge_station_pose_action.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../robot_custom_behaviour_tree_nodes/src/navigate_to_charge_station_action.cpp
)

ament_target_dependencies(monitor_bt_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  ament_index_cpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_behavior_tree
  nav2_msgs
)

# Install executables
install(TARGETS
  monitor_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
