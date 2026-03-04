cmake_minimum_required(VERSION 3.16)
project(robot_fgo_localization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ─── Find Dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)

# GTSAM
find_package(GTSAM REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  nav2_msgs
  nav2_util
  pcl_ros
  pcl_conversions
)

# ─── Library ──────────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/fgo_localization_node.cpp
  src/scan_matcher.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  gtsam
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "robot_fgo_localization::FGOLocalizationNode"
  EXECUTABLE fgo_localization_node
)

# ─── Install ──────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
