#include "quintic_walk/walk_ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace quintic_walk {

namespace {

constexpr std::array<double, 6> LEFT_LEG_AXIS_SIGNS = {
  -1.0,  // hip yaw
  -1.0,  // hip roll
  -1.0,   // hip pitch
  -1.0,   // knee
  1.0,  // ankle pitch
  1.0    // ankle roll
};

constexpr std::array<double, 6> RIGHT_LEG_AXIS_SIGNS = {
  -1.0,  // hip yaw
  -1.0,  // hip roll
  1.0,  // hip pitch
  1.0,  // knee
  -1.0,   // ankle pitch
  1.0    // ankle roll
};

}  // namespace

WalkIK::WalkIK(rclcpp::Node::SharedPtr node, walking::Params::Node::Ik config)
  : node_(node),
    config_(config),
    trunk_to_left_hip_yaw_(0.02134, 0.055, -0.090436),
    trunk_to_right_hip_yaw_(0.02134, -0.055, -0.090436),
    hip_yaw_to_pitch_x_(-0.003),
    hip_yaw_to_pitch_y_(0.0),
    hip_yaw_to_pitch_z_(-0.0335),
    hip_pitch_to_knee_(0.1168),
    knee_to_ankle_(0.1168),
    ankle_to_sole_(-0.0355),
    left_leg_joint_names_({"l_hip_yaw", "l_hip_roll", "l_hip_pitch", "l_knee", "l_ank_pitch", "l_ank_roll"}),
    right_leg_joint_names_({"r_hip_yaw", "r_hip_roll", "r_hip_pitch", "r_knee", "r_ank_pitch", "r_ank_roll"}),
    command_joint_names_({
      "l_ank_roll", "l_ank_pitch", "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw",
      "r_ank_roll", "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw"})
{
  tf2::toMsg(tf2::Transform::getIdentity(), last_left_foot_pose_);
  tf2::toMsg(tf2::Transform::getIdentity(), last_right_foot_pose_);
}

void WalkIK::init()
{
  if (config_.reset)
    reset();
}

void WalkIK::setConfig(walking::Params::Node::Ik config)
{
  config_ = config;
}

quintic_splines::JointGoals WalkIK::calculate(const WalkResponse &ik_goals)
{

  tf2::Transform trunk_to_support_foot_goal = ik_goals.support_foot_to_trunk.inverse();
  // RCLCPP_INFO(node_->get_logger(), "Support foot origin: x=%.3f y=%.3f z=%.3f",
  //           trunk_to_support_foot_goal.getOrigin().x(),
  //           trunk_to_support_foot_goal.getOrigin().y(),
  //           trunk_to_support_foot_goal.getOrigin().z());
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * ik_goals.support_foot_to_flying_foot;

  tf2::Transform trunk_to_left = ik_goals.is_left_support_foot ? trunk_to_support_foot_goal : trunk_to_flying_foot_goal;
  tf2::Transform trunk_to_right = ik_goals.is_left_support_foot ? trunk_to_flying_foot_goal : trunk_to_support_foot_goal;

  std::array<double, 6> left_leg{};
  std::array<double, 6> right_leg{};

  bool success = true;
  tf2::Transform goal;

  if (ik_goals.is_left_support_foot)
  {
    goal = trunk_to_support_foot_goal;
    success = success && solveLegIK(goal, true, left_leg);

    goal = trunk_to_flying_foot_goal;
    success = success && solveLegIK(goal, false, right_leg);
  }
  else
  {
    goal = trunk_to_flying_foot_goal;
    success = success && solveLegIK(goal, true, left_leg);

    goal = trunk_to_support_foot_goal;
    success = success && solveLegIK(goal, false, right_leg);
  }

  if (!success)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Analytic IK failed for one or both legs");
    return {};
  }

  updateCachedFootPoses(trunk_to_left, trunk_to_right);

  quintic_splines::JointGoals result;
  result.first = command_joint_names_;
  result.second = composeCommandTargets(left_leg, right_leg);
  return result;
}

void WalkIK::reset()
{
  // Nothing to reset for the analytic solver yet.
}

bool WalkIK::solveLegIK(const tf2::Transform &trunk_to_foot, bool is_left_leg, std::array<double, 6> &joint_positions)
{
  tf2::Transform goal = trunk_to_foot;
  const tf2::Vector3 hip_offset = is_left_leg ? trunk_to_left_hip_yaw_ : trunk_to_right_hip_yaw_;
  goal.setOrigin(goal.getOrigin() - hip_offset);

  double foot_roll, foot_pitch, foot_yaw;
  tf2::Matrix3x3(goal.getRotation()).getRPY(foot_roll, foot_pitch, foot_yaw);
  double hip_yaw = foot_yaw;

  tf2::Vector3 hip_yaw_to_pitch(hip_yaw_to_pitch_x_, hip_yaw_to_pitch_y_, hip_yaw_to_pitch_z_);
  hip_yaw_to_pitch = hip_yaw_to_pitch.rotate(tf2::Vector3(0.0, 0.0, 1.0), foot_yaw);
  goal.setOrigin(goal.getOrigin() - hip_yaw_to_pitch);

  tf2::Transform goal_to_ankle;
  goal_to_ankle.setIdentity();
  goal_to_ankle.setOrigin(tf2::Vector3(0.0, 0.0, -ankle_to_sole_));
  tf2::Transform to_ankle = goal * goal_to_ankle;
  tf2::Vector3 ankle_goal = to_ankle.getOrigin();

  double leg_length = ankle_goal.length();
  if ((hip_pitch_to_knee_ + knee_to_ankle_) <= leg_length)
  {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Analytic IK: target unreachable");
    return false;
  }

  double hip_roll = std::atan2(ankle_goal.z(), ankle_goal.y()) + M_PI / 2.0;
  double ankle_roll = -hip_roll + foot_roll;

  double hip_pitch = std::atan2(ankle_goal.z(), ankle_goal.x()) + M_PI / 2.0;
  double ankle_pitch = -hip_pitch - foot_pitch;

  double cos_argument = (std::pow(hip_pitch_to_knee_, 2) + std::pow(knee_to_ankle_, 2) - std::pow(leg_length, 2)) /
                        (2.0 * hip_pitch_to_knee_ * knee_to_ankle_);
  cos_argument = std::clamp(cos_argument, -1.0, 1.0);
  double beta = std::acos(cos_argument);
  double knee = (M_PI - beta) * -1.0;

  hip_pitch -= knee * 0.5;
  ankle_pitch -= knee * 0.5;

  joint_positions = { hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll };

  const auto &axis_signs = is_left_leg ? LEFT_LEG_AXIS_SIGNS : RIGHT_LEG_AXIS_SIGNS;
  for (std::size_t idx = 0; idx < joint_positions.size(); ++idx)
  {
    joint_positions[idx] *= axis_signs[idx];
  }
  return true;
}

std::vector<double> WalkIK::composeCommandTargets(const std::array<double, 6> &left_leg,
                                                  const std::array<double, 6> &right_leg) const
{
  std::vector<double> command_positions;
  command_positions.reserve(12);

  // Order: l_ank_roll, l_ank_pitch, l_knee, l_hip_pitch, l_hip_roll, l_hip_yaw
  command_positions.push_back(left_leg[5]);
  command_positions.push_back(left_leg[4]);
  command_positions.push_back(left_leg[3]);
  command_positions.push_back(left_leg[2]);
  command_positions.push_back(left_leg[1]);
  command_positions.push_back(left_leg[0]);

  // Order: r_ank_roll, r_ank_pitch, r_knee, r_hip_pitch, r_hip_roll, r_hip_yaw
  command_positions.push_back(right_leg[5]);
  command_positions.push_back(right_leg[4]);
  command_positions.push_back(right_leg[3]);
  command_positions.push_back(right_leg[2]);
  command_positions.push_back(right_leg[1]);
  command_positions.push_back(right_leg[0]);

  return command_positions;
}

void WalkIK::updateCachedFootPoses(const tf2::Transform &trunk_to_left, const tf2::Transform &trunk_to_right)
{
  tf2::toMsg(trunk_to_left, last_left_foot_pose_);
  tf2::toMsg(trunk_to_right, last_right_foot_pose_);
}

geometry_msgs::msg::Pose WalkIK::getLastLeftFootPose() const
{
  return last_left_foot_pose_;
}

geometry_msgs::msg::Pose WalkIK::getLastRightFootPose() const
{
  return last_right_foot_pose_;
}

const std::vector<std::string> &WalkIK::getLeftLegJointNames()
{
  return left_leg_joint_names_;
}

const std::vector<std::string> &WalkIK::getRightLegJointNames()
{
  return right_leg_joint_names_;
}

}  // namespace quintic_walk
