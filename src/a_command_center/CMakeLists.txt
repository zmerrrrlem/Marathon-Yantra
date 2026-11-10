cmake_minimum_required(VERSION 3.8)
project(a_command_center)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)
find_package(robot_localization REQUIRED)

ament_package()
