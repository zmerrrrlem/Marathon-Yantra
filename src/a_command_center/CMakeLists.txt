cmake_minimum_required(VERSION 3.16)
project(command_center LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(PkgConfig REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  altair_msgs
)

# Find libserial
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)


# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()


add_executable(button_handler
  src/button_handler.cpp
)
target_compile_features(button_handler PUBLIC cxx_std_17)
target_include_directories(button_handler PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(button_handler PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(button_handler PUBLIC
  ${LIBSERIAL_LIBRARIES}
)

# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# target_include_directories(${PROJECT_NAME} PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# target_link_libraries(${PROJECT_NAME} PUBLIC
#   ${LIBSERIAL_LIBRARIES}
# )
  
# INSTALL

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

# install(TARGETS ${PROJECT_NAME}
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

install(
  TARGETS button_handler
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
