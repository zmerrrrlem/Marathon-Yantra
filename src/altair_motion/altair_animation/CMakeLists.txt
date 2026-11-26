cmake_minimum_required(VERSION 3.8)
project(altair_animation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
  yaml-cpp
  altair_msgs
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
  altair_animation SHARED
  src/altair_animation.cpp
  )

target_compile_features(altair_animation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(altair_animation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

generate_parameter_library(altair_animation_parameters src/altair_animation_parameters.yaml)

target_link_libraries(altair_animation PUBLIC rclcpp::rclcpp altair_animation_parameters yaml-cpp::yaml-cpp
)
ament_target_dependencies(altair_animation PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface altair_animation_plugin.xml)


install(
  DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS altair_animation altair_animation_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  altair_animation
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
