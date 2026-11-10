#ifndef CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_IK_H_
#define CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_IK_H_
#include <moveit/robot_state/robot_state.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cubic_walk/walk_utils.hpp"
#include "cubic_walk_parameters.hpp"
#include "cubic_splines/abstract_ik.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <Eigen/Dense>
namespace cubic_walk {

class WalkIK : public cubic_splines::AbstractIK<WalkResponse> {
 public:
  explicit WalkIK(rclcpp_lifecycle::LifecycleNode::SharedPtr node, walking::Params::Node::Ik config);

  cubic_splines::JointGoals calculate(const WalkResponse& ik_goals);
  std::vector<std::pair<std::string, double>> getArmSwingGoals(double phase, double amplitude_pitch, double amplitude_roll, double elbow_bend);
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
}  // namespace cubic_walk
#endif  // CUBIC_WALK_INCLUDE_CUBIC_WALK_WALK_IK_H_
