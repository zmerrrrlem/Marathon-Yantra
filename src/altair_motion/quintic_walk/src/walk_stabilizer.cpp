#include "quintic_walk/walk_stabilizer.hpp"

namespace quintic_walk
{

WalkStabilizer::WalkStabilizer(rclcpp::Node::SharedPtr node)
  : node_(node)
  , pid_trunk_fused_pitch_(node, "node.trunk_pid.pitch", "", false)
  , pid_trunk_fused_roll_(node, "node.trunk_pid.roll", "", false)
{
  pid_trunk_fused_pitch_.initialize_from_ros_parameters();
  pid_trunk_fused_roll_.initialize_from_ros_parameters();

  reset();
}

void WalkStabilizer::reset()
{
  pid_trunk_fused_pitch_.reset();
  pid_trunk_fused_roll_.reset();
}

altair_msgs::msg::Stabilizer WalkStabilizer::getStabilizerData()
{
    altair_msgs::msg::Stabilizer msg;
    msg.timestamp   = node_->now().seconds();
    msg.pitch_error = last_pitch_error;
    msg.pitch_corr  = last_pitch_corr;
    msg.roll_error  = last_roll_error;
    msg.roll_corr   = last_roll_corr;
    return msg;
}

WalkResponse WalkStabilizer::stabilize(const WalkResponse& response, const rclcpp::Duration& dt)
{
  // compute orientation with PID control
  double goal_pitch, goal_roll, goal_yaw;
  tf2::Matrix3x3(response.support_foot_to_trunk.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);

  tf2::Quaternion quat_msg = response.support_foot_to_trunk.getRotation();
  Eigen::Quaterniond goal_orientation_eigen =
      Eigen::Quaterniond(quat_msg.getW(), quat_msg.getX(), quat_msg.getY(), quat_msg.getZ());

  // compute orientation with fused angles for PID control
  rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

  // Error
  last_pitch_error = goal_fused.fusedPitch - response.current_fused_pitch;
  last_roll_error  = goal_fused.fusedRoll  - response.current_fused_roll;

  // PID output
  last_pitch_corr = pid_trunk_fused_pitch_.compute_command(last_pitch_error, dt);
  last_roll_corr  = pid_trunk_fused_roll_.compute_command(last_roll_error, dt);

  // Change trunk x offset (in the trunks frame of reference) based on the PID controllers
  WalkResponse stabilized_response{ response };

  tf2::Transform trunk_offset;
  trunk_offset.setOrigin(tf2::Vector3(last_pitch_corr, last_roll_corr, 0.0));
  trunk_offset.setRotation(tf2::Quaternion::getIdentity());

  // apply the trunk offset to the trunk pose
  stabilized_response.support_foot_to_trunk = trunk_offset * response.support_foot_to_trunk;

  return stabilized_response;
}
}  // namespace quintic_walk
