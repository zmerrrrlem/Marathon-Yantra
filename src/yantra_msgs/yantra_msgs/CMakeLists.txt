cmake_minimum_required(VERSION 3.8)
project(yantra_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Audio.msg"
  "msg/Buttons.msg"
  "msg/Cpu.msg"
  "msg/Filesystem.msg"
  "msg/FootPressure.msg"
  "msg/HeadMode.msg"
  "msg/JointCommand.msg"
  "msg/JointTorque.msg"
  "msg/NetworkInterface.msg"
  "msg/PoseWithCertainty.msg"
  "msg/PoseWithCertaintyArray.msg"
  "msg/RobotControlState.msg"
  "msg/RobotRelative.msg"
  "msg/RobotRelativeArray.msg"
  "msg/Strategy.msg"
  "msg/TeamData.msg"
  "msg/Workload.msg"
  "msg/WalkDebug.msg"
  "msg/WalkEngineDebug.msg"
  "srv/YantraAnimation.srv"
  "msg/ImageSub.msg"
  "msg/BallInfo.msg"
  "msg/GameState.msg"
  
  DEPENDENCIES
  builtin_interfaces
  action_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs)


ament_package()