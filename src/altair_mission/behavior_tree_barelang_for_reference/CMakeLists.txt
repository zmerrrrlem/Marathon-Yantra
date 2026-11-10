cmake_minimum_required(VERSION 3.16)
project(behavior_tree LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_geometry_msgs
  biped_interfaces
  altair_msgs
  ball_detect
  altair_msgs
  behaviortree_cpp
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

add_executable(behavior_tree
  src/main.cpp
)

target_include_directories(behavior_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_tree
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# INSTALL

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_package()
