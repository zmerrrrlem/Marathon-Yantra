#ifndef QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_STABILIZER_H_
#define QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_STABILIZER_H_


#include "rot_conv/rot_conv/rot_conv.h"

#include <Eigen/Geometry>
#include <quintic_splines/abstract_stabilizer.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "quintic_walk/walk_utils.hpp"
#include "quintic_splines/abstract_stabilizer.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace quintic_walk {

class WalkStabilizer : public quintic_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const rclcpp::Duration &dt) override;

 private:
  control_toolbox::PidROS pid_trunk_fused_pitch_;
  control_toolbox::PidROS pid_trunk_fused_roll_;
};
}  // namespace quintic_walk

#endif  // QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_STABILIZER_H_
