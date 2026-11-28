#ifndef QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_IK_H_
#define QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_IK_H_
#include <moveit/robot_state/robot_state.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "quintic_walk/walk_utils.hpp"
#include "quintic_walk_parameters.hpp"
#include "quintic_splines/abstract_ik.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <Eigen/Dense>
namespace quintic_walk {

class WalkIK : public quintic_splines::AbstractIK<WalkResponse> {
 public:
  explicit WalkIK(rclcpp_lifecycle::LifecycleNode::SharedPtr node, walking::Params::Node::Ik config);

  quintic_splines::JointGoals calculate(const WalkResponse& ik_goals);
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  void setConfig(walking::Params::Node::Ik config);

  const std::vector<std::string>& getLeftLegJointNames();
  const std::vector<std::string>& getRightLegJointNames();

  moveit::core::RobotStatePtr get_goal_state();

  // Number of consecutive cycles in which at least one leg failed IK (0 = last cycle was fine)
  int getConsecutiveFailures() const { return consecutive_failures_; }

 private:
  // Geometry of one leg, measured from the robot model at init (zero joint configuration).
  struct LegGeometry {
    bool valid = false;
    tf2::Vector3 hip_in_root;        // hip-pitch joint origin, root frame
    tf2::Vector3 tip_to_ankle;       // ankle-pitch joint origin expressed in the IK tip frame
    double max_reach = 0.0;          // thigh + shank length
  };

  LegGeometry measureLeg(const moveit::core::JointModelGroup* leg_group);
  geometry_msgs::msg::Pose clampToReach(const geometry_msgs::msg::Pose& goal, const LegGeometry& leg, bool& clamped) const;
  bool solveLeg(const moveit::core::JointModelGroup* leg_group, const geometry_msgs::msg::Pose& goal);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  LegGeometry left_leg_geometry_;
  LegGeometry right_leg_geometry_;
  int consecutive_failures_ = 0;

  std::array<double, 12> joint_axis_direction_;

  moveit::core::RobotStatePtr goal_state_;
  const moveit::core::JointModelGroup* legs_joints_group_ = nullptr;
  const moveit::core::JointModelGroup* left_leg_joints_group_ = nullptr;
  const moveit::core::JointModelGroup* right_leg_joints_group_ = nullptr;
  walking::Params::Node::Ik config_;
};
}  // namespace quintic_walk
#endif  // QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_IK_H_
