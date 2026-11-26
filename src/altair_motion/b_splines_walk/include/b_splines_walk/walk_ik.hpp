#ifndef CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_IK_H_
#define CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_IK_H_
#include <moveit/robot_state/robot_state.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "b_splines_walk/walk_utils.hpp"
#include "b_splines_walk_parameters.hpp"
#include "b_splines/abstract_ik.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <Eigen/Dense>
namespace b_splines_walk {

class WalkIK : public b_splines::AbstractIK<WalkResponse> {
 public:
  explicit WalkIK(rclcpp_lifecycle::LifecycleNode::SharedPtr node, walking::Params::Node::Ik config);

  b_splines::JointGoals calculate(const WalkResponse& ik_goals);
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  void setConfig(walking::Params::Node::Ik config);

  const std::vector<std::string>& getLeftLegJointNames();
  const std::vector<std::string>& getRightLegJointNames();

  moveit::core::RobotStatePtr get_goal_state();

 private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  std::array<double, 12> joint_axis_direction_;

  moveit::core::RobotStatePtr goal_state_;
  const moveit::core::JointModelGroup* legs_joints_group_;
  const moveit::core::JointModelGroup* left_leg_joints_group_;
  const moveit::core::JointModelGroup* right_leg_joints_group_;
  walking::Params::Node::Ik config_;
};
}  // namespace b_splines_walk
#endif  // CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_IK_H_
