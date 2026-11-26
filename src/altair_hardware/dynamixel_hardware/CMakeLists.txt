cmake_minimum_required(VERSION 3.16)
project(dynamixel_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  parameter_traits
  realtime_tools
  std_msgs
  sensor_msgs
  dynamixel_workbench_toolbox
  dynamixel_msgs
  std_srvs
)

# Find libserial
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
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

# hardware interface ================
add_library(
  ${PROJECT_NAME}
  SHARED
  src/dynamixel_hardware.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# include hardware
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${LIBSERIAL_LIBRARIES}
)
# ================================

# for testing ================
add_executable(test_sync_read src/test_sync_read.cpp)
target_compile_features(test_sync_read PUBLIC cxx_std_17)
target_include_directories(test_sync_read PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(test_sync_read PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(test_sync_read PUBLIC
  ${LIBSERIAL_LIBRARIES}
)
# ==========================

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface altair_hardware_interface_plugin.xml)
  
# INSTALL

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install test_sync_read executable
install(
  TARGETS test_sync_read
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
