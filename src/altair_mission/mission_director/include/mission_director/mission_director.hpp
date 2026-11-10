#pragma once

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "altair_msgs/msg/robot_control_state.hpp"

class MissionDirector
{
public:
  MissionDirector();
  ~MissionDirector();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr service_node_;
  void run();

private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> publisher_;

  sensor_msgs::msg::Imu imu_msg_;
  altair_msgs::msg::RobotControlState robot_state_msg_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<altair_msgs::msg::RobotControlState>::SharedPtr robot_state_pub_;
  rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_module_client_;
  std::shared_ptr<robotis_controller_msgs::srv::SetModule::Request> set_module_;

  BT::NodeStatus set_module();

  void set_robot_state(const int state);
  BT::NodeStatus set_robot_state_controllable();
  BT::NodeStatus set_robot_state_falling();
  BT::NodeStatus set_robot_state_fallen();
  BT::NodeStatus set_robot_state_getting_up();
  BT::NodeStatus set_robot_state_animation_running();
  BT::NodeStatus set_robot_state_penalty();
  BT::NodeStatus set_robot_state_walking_init();
  BT::NodeStatus set_robot_state_walking();
  BT::NodeStatus set_robot_state_motor_off();
  BT::NodeStatus set_robot_state_picked_up();
  BT::NodeStatus set_robot_state_kicking();

  rclcpp::TimerBase::SharedPtr runner_;
};
