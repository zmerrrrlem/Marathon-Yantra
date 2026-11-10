#ifndef QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_STABILIZER_H_
#define QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_STABILIZER_H_


#include "rot_conv/rot_conv/rot_conv.h"

#include <Eigen/Geometry>
#include <quintic_splines/abstract_stabilizer.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "quintic_walk/walk_utils.hpp"
#include "quintic_splines/abstract_stabilizer.hpp"

#include "altair_msgs/msg/stabilizer.hpp"

namespace quintic_walk {

class WalkStabilizer : public quintic_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(rclcpp::Node::SharedPtr node);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const rclcpp::Duration &dt) override;
  altair_msgs::msg::Stabilizer getStabilizerData();

 private:
  rclcpp::Node::SharedPtr node_;  // simpan node
  control_toolbox::PidROS pid_trunk_fused_pitch_;
  control_toolbox::PidROS pid_trunk_fused_roll_;

  double last_pitch_error{0.0};
  double last_roll_error{0.0};
  double last_pitch_corr{0.0};
  double last_roll_corr{0.0};
};
}  // namespace quintic_walk

#endif  // QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_STABILIZER_H_
