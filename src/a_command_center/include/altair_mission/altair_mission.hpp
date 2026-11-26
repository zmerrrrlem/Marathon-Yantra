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

namespace altair_mission {



class AltairMission : public rclcpp::Node {

 public:

  AltairMission();

 private:

  BT::NodeStatus ballfound();
  
};


}
