#ifndef QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_IK_H_
#define QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_IK_H_
#include <array>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "quintic_walk/walk_utils.hpp"
#include "quintic_walk_parameters.hpp"
#include "quintic_splines/abstract_ik.hpp"
#include <rclcpp/rclcpp.hpp>
namespace quintic_walk {

class WalkIK : public quintic_splines::AbstractIK<WalkResponse> {
 public:
  explicit WalkIK(rclcpp::Node::SharedPtr node, walking::Params::Node::Ik config);

  quintic_splines::JointGoals calculate(const WalkResponse& ik_goals);
  void init() override;
  void reset() override;
  void setConfig(walking::Params::Node::Ik config);

  const std::vector<std::string>& getLeftLegJointNames();
  const std::vector<std::string>& getRightLegJointNames();

  geometry_msgs::msg::Pose getLastLeftFootPose() const;
  geometry_msgs::msg::Pose getLastRightFootPose() const;

 private:
  bool solveLegIK(const tf2::Transform& trunk_to_foot, bool is_left_leg, std::array<double, 6>& joint_positions);
  std::vector<double> composeCommandTargets(const std::array<double, 6>& left_leg,
                                            const std::array<double, 6>& right_leg) const;
  void updateCachedFootPoses(const tf2::Transform& trunk_to_left, const tf2::Transform& trunk_to_right);

  rclcpp::Node::SharedPtr node_;

  walking::Params::Node::Ik config_;
  tf2::Vector3 trunk_to_left_hip_yaw_;
  tf2::Vector3 trunk_to_right_hip_yaw_;

  double hip_yaw_to_pitch_x_;
  double hip_yaw_to_pitch_y_;
  double hip_yaw_to_pitch_z_;
  double hip_pitch_to_knee_;
  double knee_to_ankle_;
  double ankle_to_sole_;

  std::vector<std::string> left_leg_joint_names_;
  std::vector<std::string> right_leg_joint_names_;
  std::vector<std::string> command_joint_names_;

  geometry_msgs::msg::Pose last_left_foot_pose_;
  geometry_msgs::msg::Pose last_right_foot_pose_;
};
}  // namespace quintic_walk
#endif  // QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_IK_H_
