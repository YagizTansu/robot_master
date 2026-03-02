cmake_minimum_required(VERSION 3.10)
project(robot_custom_layers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

set(dependencies
  rclcpp
  nav2_costmap_2d
  pluginlib
  nav2_util
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  robot_interfaces
)

# Include directories
include_directories(
  include
)

# Prohibited Layer library
add_library(prohibited_layer SHARED
  src/prohibited_layer.cpp
)

ament_target_dependencies(prohibited_layer ${dependencies})

# Install
install(TARGETS prohibited_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

install(FILES prohibited_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export
pluginlib_export_plugin_description_file(nav2_costmap_2d prohibited_layer_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(prohibited_layer)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
