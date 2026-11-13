cmake_minimum_required(VERSION 3.8)
project(altair_odometry_fuser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
