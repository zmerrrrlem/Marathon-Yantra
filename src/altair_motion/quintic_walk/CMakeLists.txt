cmake_minimum_required(VERSION 3.8)
project(quintic_walk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  altair_msgs
  biped_interfaces
  control_toolbox
  dynamixel_sdk
  generate_parameter_library
  geometry_msgs
  nav_msgs
  parameter_traits
  quintic_splines
  rclcpp
  robotis_device
  robotis_framework_common
  robotis_math
  rot_conv
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  pinocchio
)
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

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

generate_parameter_library(quintic_walk_parameters src/quintic_walk_parameters.yaml)

add_library(${PROJECT_NAME}
  src/main.cpp
  src/quintic_walk.cpp
  src/walk_visualizer.cpp
  src/walk_engine.cpp
  src/walk_stabilizer.cpp
  src/walk_ik.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  quintic_walk_parameters
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(plot_trajectory
  test/plot_trajectory.cpp
)
target_compile_features(plot_trajectory PUBLIC cxx_std_20)
target_include_directories(plot_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(plot_trajectory
  ${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(plot_trajectory
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(DIRECTORY config test
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} quintic_walk_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS plot_trajectory
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/show_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
