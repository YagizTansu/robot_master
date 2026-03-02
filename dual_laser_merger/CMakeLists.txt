cmake_minimum_required(VERSION 3.10)

# Set CMake policies BEFORE project() to ensure they apply globally
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)  # Suppress FindBoost warnings from PCL
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 OLD)  # Suppress FLANN_ROOT warnings from PCL
endif()

project(dual_laser_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(dual_laser_merger SHARED
  src/dual_laser_merger.cpp)

rclcpp_components_register_node(dual_laser_merger
  PLUGIN "merger_node::MergerNode"
  EXECUTABLE dual_laser_merger_node)

install(DIRECTORY include DESTINATION include/dual_laser_merger)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bag DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_flake8_FOUND TRUE)
  set(ament_pep257_FOUND TRUE)
  set(ament_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)