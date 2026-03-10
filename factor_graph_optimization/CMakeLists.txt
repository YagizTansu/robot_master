cmake_minimum_required(VERSION 3.10)
project(factor_graph_optimization)

# ── C++17 ─────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 / ament ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# TF2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# LiDAR projection
find_package(laser_geometry REQUIRED)

# PCL bridging (ROS wrappers)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# ── Third-party ───────────────────────────────────────────────────────────────
# GTSAM (installed system-wide or via colcon overlay)
find_package(GTSAM REQUIRED)

# PCL (comes with pcl_ros, but explicit for NDT/ICP)
find_package(PCL REQUIRED COMPONENTS common io registration filters)

# Eigen (pulled in by GTSAM, declared explicitly for clarity)
find_package(Eigen3 REQUIRED)

# ── Include dirs ──────────────────────────────────────────────────────────────
include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ── fgo_node executable ───────────────────────────────────────────────────────
add_executable(fgo_node
  src/fgo_main.cpp
  src/fgo_node.cpp
  src/core/pose_conversion.cpp
  src/core/noise_model.cpp
  src/core/geometry_2d.cpp
)

target_link_libraries(fgo_node
  gtsam
  ${PCL_LIBRARIES}
)

ament_target_dependencies(fgo_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
)

# ── scan_matcher_node executable ──────────────────────────────────────────────
add_executable(scan_matcher_node
  src/scan_matcher_main.cpp
  src/scan_matcher.cpp
  src/core/geometry_2d.cpp
)

target_link_libraries(scan_matcher_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(scan_matcher_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  laser_geometry
  pcl_ros
  pcl_conversions
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  fgo_node
  scan_matcher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
