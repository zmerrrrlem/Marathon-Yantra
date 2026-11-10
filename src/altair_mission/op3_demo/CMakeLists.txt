cmake_minimum_required(VERSION 3.8)
project(op3_demo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(op3_action_module_msgs REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(altair_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# Include directories
include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

# Soccer Demo Node
add_executable(op3_demo_node 
  src/demo_node.cpp
  src/soccer_demo.cpp
)

ament_target_dependencies(op3_demo_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_action_module_msgs
  robotis_math
  Eigen3
  altair_msgs
  tf2
  tf2_ros
)

target_link_libraries(op3_demo_node
  ${YAML_CPP_LIBRARIES}
)

# Install executables
install(TARGETS op3_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/ball_position_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robotis_controller_msgs
  op3_action_module_msgs
  robotis_math
  Eigen3
  altair_msgs
  tf2
  tf2_ros
)

ament_package()
