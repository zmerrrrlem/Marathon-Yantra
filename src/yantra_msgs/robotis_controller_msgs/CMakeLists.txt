cmake_minimum_required(VERSION 3.8)
project(robotis_controller_msgs)

find_package(ament_cmake REQUIRED COMPONENTS)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCtrlModule.msg"
  "msg/StatusMsg.msg"
  "msg/SyncWriteItem.msg"
  "msg/WriteControlTable.msg"
  "srv/GetJointModule.srv"
  "srv/LoadOffset.srv"
  "srv/SetJointModule.srv"
  "srv/SetModule.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()