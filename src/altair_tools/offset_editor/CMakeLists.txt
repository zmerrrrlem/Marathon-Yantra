cmake_minimum_required(VERSION 3.8)
project(offset_editor LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp
  rclcpp_lifecycle
  parameter_traits
  realtime_tools
  std_msgs
  sensor_msgs
  dynamixel_msgs
  std_srvs
  ament_index_cpp
  Curses
  yaml-cpp
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

add_executable(
  ${PROJECT_NAME}
  src/offset_editor.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# include hardware
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${CURSES_LIBRARIES}
  yaml-cpp::yaml-cpp
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_package()
