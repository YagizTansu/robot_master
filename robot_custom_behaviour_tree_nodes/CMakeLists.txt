cmake_minimum_required(VERSION 3.10)
project(robot_custom_behaviour_tree_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(
  include
)

# Build graph_based_planner library
set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_core
  nav2_util
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
  nav2_behavior_tree
  sensor_msgs
  std_msgs
)

# Build Battery Check BT Node
add_library(battery_check_action_bt_node SHARED
  src/battery_check_action.cpp
)

ament_target_dependencies(battery_check_action_bt_node
  ${dependencies}
)

target_compile_definitions(battery_check_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS battery_check_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build Get Charge Station Pose BT Node
add_library(get_charge_station_pose_action_bt_node SHARED
  src/get_charge_station_pose_action.cpp
)

ament_target_dependencies(get_charge_station_pose_action_bt_node
  ${dependencies}
)

target_compile_definitions(get_charge_station_pose_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS get_charge_station_pose_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build Emergency Stop BT Node
add_library(emergency_stop_action_bt_node SHARED
  src/emergency_stop_action.cpp
)

ament_target_dependencies(emergency_stop_action_bt_node
  ${dependencies}
)

target_compile_definitions(emergency_stop_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS emergency_stop_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Build Navigate To Charge Station BT Node
add_library(navigate_to_charge_station_action_bt_node SHARED
  src/navigate_to_charge_station_action.cpp
)

ament_target_dependencies(navigate_to_charge_station_action_bt_node
  ${dependencies}
)

target_compile_definitions(navigate_to_charge_station_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS navigate_to_charge_station_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin description file
pluginlib_export_plugin_description_file(nav2_behavior_tree battery_check_plugin.xml)
pluginlib_export_plugin_description_file(nav2_behavior_tree get_charge_station_pose_plugin.xml)
pluginlib_export_plugin_description_file(nav2_behavior_tree emergency_stop_plugin.xml)
pluginlib_export_plugin_description_file(nav2_behavior_tree navigate_to_charge_station_plugin.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies and include directories
ament_export_include_directories(include)
ament_export_libraries(
  battery_check_action_bt_node
  emergency_stop_action_bt_node
  get_charge_station_pose_action_bt_node
  navigate_to_charge_station_action_bt_node
)
ament_export_dependencies(
  rclcpp
  behaviortree_cpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  nav2_behavior_tree
)

ament_package()
