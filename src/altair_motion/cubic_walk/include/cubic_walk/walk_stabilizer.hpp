#ifndef CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_STABILIZER_H_
#define CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_STABILIZER_H_


#include "rot_conv/rot_conv/rot_conv.h"

#include <Eigen/Geometry>
#include <cubic_splines/abstract_stabilizer.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cubic_walk/walk_utils.hpp"
#include "cubic_splines/abstract_stabilizer.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cubic_walk {

class WalkStabilizer : public cubic_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const rclcpp::Duration &dt) override;

 private:
  control_toolbox::PidROS pid_trunk_fused_pitch_;
  control_toolbox::PidROS pid_trunk_fused_roll_;
};
}  // namespace cubic_walk

#endif  // CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_STABILIZER_H_
