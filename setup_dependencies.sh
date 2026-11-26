#!/bin/bash
# System updates and core ROS 2 build tools
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep ros-jazzy-ament-package libncurses-dev

# Dynamixel SDK and Hardware dependencies
sudo apt install -y ros-jazzy-dynamixel-sdk libserial-dev libserialport-dev libserial1

# ROS 2 Control & MoveIt2 dependencies
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-hardware-interface ros-jazzy-forward-command-controller ros-jazzy-filters ros-jazzy-controller-manager ros-jazzy-controller-manager-msgs ros-jazzy-control-toolbox ros-jazzy-controller-interface ros-jazzy-ros2controlcli ros-jazzy-ros2component ros-jazzy-pid-controller ros-jazzy-joint-state-broadcaster ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim
sudo apt install -y ros-jazzy-moveit ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-cyclonedds ros-jazzy-moveit-ros-control-interface ros-jazzy-ompl

# Miscellaneous ROS 2 utilities
sudo apt install -y ros-jazzy-xacro ros-jazzy-rqt ros-jazzy-rviz2 ros-jazzy-yaml-cpp-vendor ros-jazzy-vision-msgs ros-jazzy-nav-msgs ros-jazzy-orocos-kdl-vendor ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-joint-limits ros-jazzy-interactive-markers ros-jazzy-image-geometry ros-jazzy-imu-sensor-broadcaster ros-jazzy-kinematics-interface ros-jazzy-libyaml-vendor ros-jazzy-examples-rclcpp-minimal-subscriber ros-jazzy-examples-rclcpp-minimal-publisher
