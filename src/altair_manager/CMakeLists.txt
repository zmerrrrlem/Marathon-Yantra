cmake_minimum_required(VERSION 3.5)
project(altair_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_controller REQUIRED)
find_package(robotis_math REQUIRED)
# find_package(subcon_module REQUIRED)
find_package(op3_base_module REQUIRED)
find_package(experiment_module REQUIRED)
find_package(op3_action_module REQUIRED)
find_package(quintic_walk REQUIRED)
find_package(altair_walk_module REQUIRED)
find_package(robot_dynamics REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
  ${robotis_controller_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  # ${subcon_module_INCLUDE_DIRS}
  ${op3_base_module_INCLUDE_DIRS}
  ${experiment_module_INCLUDE_DIRS}
  ${op3_action_module_INCLUDE_DIRS}
  ${quintic_walk_INCLUDE_DIRS}
  ${altair_walk_module_INCLUDE_DIRS}
  ${robot_dynamics_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/altair_manager.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  robotis_controller_msgs
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  # subcon_module
  op3_base_module
  experiment_module
  op3_action_module
  quintic_walk
  altair_walk_module
  robot_dynamics
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
