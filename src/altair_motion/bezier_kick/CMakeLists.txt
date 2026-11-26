cmake_minimum_required(VERSION 3.8)
project(bezier_kick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wall -Wno-unused -Wextra -Wpedantic)

find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package(Boost COMPONENTS python3 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(ros2_python_extension REQUIRED)
find_package(pybind11 REQUIRED)

find_package(PkgConfig REQUIRED)
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
  altair_msgs
  altair_splines
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

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
"Are you using the correct branch of the ros2_control_demos repository?")
endif()
# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

add_library(bezier_kick SHARED
  src/bezier_kick.cpp
  src/kick_ik.cpp
)
# add_compile_options(-std=c++20)
target_compile_features(bezier_kick PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

target_include_directories(bezier_kick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

generate_parameter_library(bezier_kick_parameters src/bezier_kick_parameters.yaml)

target_link_libraries(bezier_kick PUBLIC rclcpp::rclcpp bezier_kick_parameters)
ament_target_dependencies(bezier_kick PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface bezier_kick_plugin.xml)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS bezier_kick bezier_kick_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
  )
ament_export_libraries(bezier_kick)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
