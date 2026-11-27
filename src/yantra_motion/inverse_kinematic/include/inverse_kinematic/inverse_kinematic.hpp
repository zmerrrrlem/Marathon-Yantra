#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "yantra_splines/abstract_ik.hpp"


namespace inverse_kinematic {
  class InverseKinematic {
  public:
    explicit InverseKinematic(rclcpp::Node::SharedPtr node);
    yantra_splines::JointGoals calculate(const WalkResponse& ik_goals);

  private:
    rclcpp::Node::SharedPtr node_;

    void setup();
  };
}
