#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

namespace yantra_mission {



class YantraMission : public rclcpp::Node {

 public:

  YantraMission();

 private:

  BT::NodeStatus ballfound();
  
};


}
