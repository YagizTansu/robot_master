cmake_minimum_required(VERSION 3.8)
project(pallet_detection_and_docking)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ---- pallet_detection ----
add_executable(pallet_detection src/pallet_detection.cpp)
ament_target_dependencies(pallet_detection
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  robot_interfaces
  rcl_interfaces
)
target_link_libraries(pallet_detection
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# ---- pallet_docking ----
add_executable(pallet_docking src/pallet_docking.cpp)
ament_target_dependencies(pallet_docking
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  action_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  robot_interfaces
  rcl_interfaces
  angles
)
target_link_libraries(pallet_docking
  ${EIGEN3_LIBRARIES}
)

# ---- path_follower ----
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  action_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
)

install(TARGETS
  pallet_detection
  pallet_docking
  path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params/
)

ament_package()
