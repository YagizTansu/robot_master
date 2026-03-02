cmake_minimum_required(VERSION 3.10)
project(robot_database)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MONGOCXX REQUIRED libmongocxx)

link_directories(/usr/local/lib)

add_executable(robot_database_node src/robot_database_node.cpp)
ament_target_dependencies(robot_database_node robot_interfaces rclcpp geometry_msgs)
target_link_libraries(robot_database_node ${MONGOCXX_LIBRARIES})
target_include_directories(robot_database_node PUBLIC ${MONGOCXX_INCLUDE_DIRS})

add_executable(robot_database_stream_node src/robot_database_stream_node.cpp)
ament_target_dependencies(robot_database_stream_node robot_interfaces rclcpp)
target_link_libraries(robot_database_stream_node ${MONGOCXX_LIBRARIES})
target_include_directories(robot_database_stream_node PUBLIC ${MONGOCXX_INCLUDE_DIRS})

install(TARGETS 
  robot_database_node
  robot_database_stream_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
