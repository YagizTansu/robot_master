cmake_minimum_required(VERSION 3.10)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)


install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  robot_startup_param
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
