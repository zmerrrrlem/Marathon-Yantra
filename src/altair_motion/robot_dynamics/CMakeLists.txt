cmake_minimum_required(VERSION 3.8)
project(robot_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  PkgConfig
  altair_msgs
  biped_interfaces
  control_toolbox
  generate_parameter_library
  geometry_msgs
  nav_msgs
  parameter_traits
  quintic_splines
  rclcpp
  robotis_device
  robotis_framework_common
  robotis_controller_msgs
  robotis_math
  rot_conv
  sensor_msgs
  std_msgs
  Eigen3
  pinocchio
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${robotis_math_INCLUDE_DIRS}
  ${robotis_controller_msgs_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
  ${robotis_framework_common_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/robot_dynamics.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
