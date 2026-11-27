cmake_minimum_required(VERSION 3.8)
project(inverse_kinematic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wall -Werror -Wno-unused -Wextra -Wpedantic)

find_package(PkgConfig REQUIRED)
find_package(ros2_python_extension REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  backward_ros
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  controller_interface
  generate_parameter_library
  parameter_traits
  biped_interfaces
  yantra_msgs
  yantra_splines
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_kinematics
  moveit_ros_planning
  nav_msgs
  rclcpp
  rot_conv
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  ament_index_cpp
  rosidl_default_generators
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

add_executable(inverse_kinematic
  src/inverse_kinematic.cpp
)

# add_compile_options(-std=c++20)
target_compile_features(quintic_walk PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

target_include_directories(inverse_kinematic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${TARGET_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include 
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
