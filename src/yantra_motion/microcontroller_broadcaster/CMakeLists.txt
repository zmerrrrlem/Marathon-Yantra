cmake_minimum_required(VERSION 3.8)
project(microcontroller_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Python3 COMPONENTS Development NumPy REQUIRED)
find_package(Boost COMPONENTS python3 REQUIRED)

# Include directories
include_directories(${Python3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} external/matplotlibcpp/)

find_package(PkgConfig REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  generate_parameter_library
  parameter_traits
  realtime_tools
  std_msgs
  sensor_msgs
  control_msgs
  geometry_msgs
  kdl_parser
  Eigen3
  ament_index_cpp
  Boost
  moveit_ros_planning_interface
  moveit_core
  moveit_kinematics
  moveit_ros_planning
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

add_library(
  microcontroller_broadcaster SHARED
  src/microcontroller_broadcaster.cpp
  )

target_compile_features(microcontroller_broadcaster PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(microcontroller_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

generate_parameter_library(microcontroller_broadcaster_parameters src/microcontroller_broadcaster_parameters.yaml)

target_link_libraries(microcontroller_broadcaster PUBLIC rclcpp::rclcpp microcontroller_broadcaster_parameters ${Python3_LIBRARIES} ${Boost_LIBRARIES}
)
ament_target_dependencies(microcontroller_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface microcontroller_broadcaster_plugin.xml)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS microcontroller_broadcaster microcontroller_broadcaster_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  microcontroller_broadcaster
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
