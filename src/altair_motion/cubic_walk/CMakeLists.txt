cmake_minimum_required(VERSION 3.8)
project(cubic_walk)

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
  cubic_splines
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
  srdfdom
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

add_library(cubic_walk SHARED
  src/cubic_walk.cpp
  # src/walk_node.cpp
  src/walk_visualizer.cpp 
  src/walk_engine.cpp 
  src/walk_stabilizer.cpp
  src/walk_ik.cpp
)
# add_compile_options(-std=c++20)
target_compile_features(cubic_walk PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

target_include_directories(cubic_walk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

generate_parameter_library(cubic_walk_parameters src/cubic_walk_parameters.yaml)

target_link_libraries(cubic_walk PUBLIC rclcpp::rclcpp cubic_walk_parameters yaml-cpp)
ament_target_dependencies(cubic_walk PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface cubic_walk_plugin.xml)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS cubic_walk cubic_walk_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
  )
ament_export_libraries(cubic_walk)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
