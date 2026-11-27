cmake_minimum_required(VERSION 3.5)
project(humanoid_league_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# find_package(bitbots_docs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Animation.msg"
  "msg/Audio.msg"
  "msg/GameState.msg"
  "msg/HeadMode.msg"
  "msg/LineInformationRelative.msg"
  "msg/LineIntersectionRelative.msg"
  "msg/LineSegmentRelative.msg"
  "msg/ObstacleRelativeArray.msg"
  "msg/ObstacleRelative.msg"
  "msg/PoseWithCertaintyArray.msg"
  "msg/PoseWithCertainty.msg"
  "msg/PoseWithCertaintyStamped.msg"
  "msg/RobotControlState.msg"
  "msg/Strategy.msg"
  "msg/TeamData.msg"
  "action/PlayAnimation.action"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
)

# enable_bitbots_docs()

ament_package()
