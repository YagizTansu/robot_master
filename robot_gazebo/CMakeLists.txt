cmake_minimum_required(VERSION 3.10)
project(robot_gazebo)

# C++17 required for gz transport / msgs
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ───────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Gazebo transport / msgs (bundled inside Jazzy ROS overlay) ────────────────
# Tell CMake where to find gz vendor cmake configs.
list(APPEND CMAKE_PREFIX_PATH
  "/opt/ros/jazzy/opt/gz_transport_vendor"
  "/opt/ros/jazzy/opt/gz_msgs_vendor"
  "/opt/ros/jazzy/opt/gz_cmake_vendor"
)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ── ground_truth_publisher (C++ node) ────────────────────────────────────────
add_executable(ground_truth_publisher
  src/ground_truth_publisher.cpp
)
ament_target_dependencies(ground_truth_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  builtin_interfaces
)
target_link_libraries(ground_truth_publisher
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
install(TARGETS ground_truth_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch models worlds DESTINATION share/${PROJECT_NAME})

# Python benchmark scripts
install(
  PROGRAMS
    scripts/localization_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)