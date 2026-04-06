cmake_minimum_required(VERSION 3.10)
project(robot_localization)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

set(library_name rl_lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLL.srv"
  "srv/FromLLArray.srv"
  "srv/GetState.srv"
  "srv/SetDatum.srv"
  "srv/SetPose.srv"
  "srv/SetUTMZone.srv"
  "srv/ToggleFilterProcessing.srv"
  "srv/ToLL.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
  ADD_LINTER_TESTS
)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(
  ${library_name}
  src/ekf.cpp
  src/ukf.cpp
  src/filter_base.cpp
  src/filter_utilities.cpp
  src/navsat_transform.cpp
  src/robot_localization_estimator.cpp
  src/ros_filter.cpp
  src/ros_filter_utilities.cpp
  src/ros_robot_localization_listener.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${library_name} "${cpp_typesupport_target}")

add_executable(
  ekf_node
  src/ekf_node.cpp
)

add_executable(
  ukf_node
  src/ukf_node.cpp
)

add_executable(
  navsat_transform_node
  src/navsat_transform_node.cpp
)

add_executable(
  robot_localization_listener_node
  src/robot_localization_listener_node.cpp
)

target_link_libraries(
  ${library_name}
  ${GeographicLib_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(
  ${library_name}
  angles
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
)

target_link_libraries(
  ekf_node
  ${library_name}
)

ament_target_dependencies(
  ekf_node
  rclcpp
)

target_link_libraries(
  ukf_node
  ${library_name}
)

ament_target_dependencies(
  ukf_node
  rclcpp
)

target_link_libraries(
  navsat_transform_node
  ${library_name}
)

ament_target_dependencies(
  navsat_transform_node
  rclcpp
)

target_link_libraries(
  robot_localization_listener_node
  ${library_name}
)

ament_target_dependencies(
  robot_localization_listener_node
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

endif()

install(
  TARGETS
  navsat_transform_node
  ekf_node
  ukf_node
  robot_localization_listener_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# Install params config files.
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
