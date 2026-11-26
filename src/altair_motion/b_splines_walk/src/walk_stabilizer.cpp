#include "b_splines_walk/walk_stabilizer.hpp"

namespace b_splines_walk {

  WalkStabilizer::WalkStabilizer(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : pid_trunk_fused_pitch_(node, "node.trunk_pid.pitch"), pid_trunk_fused_roll_(node, "node.trunk_pid.roll") {
    pid_trunk_fused_pitch_.initialize_from_ros_parameters();
    pid_trunk_fused_roll_.initialize_from_ros_parameters();

    // Initialize PID gains (tune these!)
    // pid_trunk_fused_roll_.initPid(0.5, 0.0, 0.1, 0.0, -1.0, 1.0);   // Kp=0.5, Kd=0.1
    // pid_trunk_fused_pitch_.initPid(0.5, 0.0, 0.1, 0.0, -1.0, 1.0);  // Kp=0.5, Kd=0.1
    // pid_ankle_roll_.initPid(0.3, 0.0, 0.05, 0.0, -0.5, 0.5);        // Smaller gains for fine-tuning
    // pid_ankle_pitch_.initPid(0.3, 0.0, 0.05, 0.0, -0.5, 0.5);
    // pid_hip_roll_.initPid(1.0, 0.0, 0.2, 0.0, -1.5, 1.5);           // More aggressive for CoM shifting
    // pid_hip_pitch_.initPid(1.0, 0.0, 0.2, 0.0, -1.5, 1.5);

    reset();
  }

  void WalkStabilizer::reset() {
    pid_trunk_fused_pitch_.reset();
    pid_trunk_fused_roll_.reset();
  }

  WalkResponse WalkStabilizer::stabilize(const WalkResponse& response, const rclcpp::Duration& dt) {
    // compute orientation with PID control
    double goal_pitch, goal_roll, goal_yaw;
    tf2::Matrix3x3(response.support_foot_to_trunk.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);

    tf2::Quaternion quat_msg = response.support_foot_to_trunk.getRotation();
    Eigen::Quaterniond goal_orientation_eigen =
      Eigen::Quaterniond(quat_msg.getW(), quat_msg.getX(), quat_msg.getY(), quat_msg.getZ());

    // compute orientation with fused angles for PID control
    rot_conv::FusedAngles goal_fused = rot_conv::FusedFromQuat(goal_orientation_eigen);

    // adapt trunk values based on PID controllers
    double fused_roll_correction =
      pid_trunk_fused_roll_.computeCommand(goal_fused.fusedRoll - response.current_fused_roll, dt);
    double fused_pitch_correction =
      pid_trunk_fused_pitch_.computeCommand(goal_fused.fusedPitch - response.current_fused_pitch, dt);

    // Change trunk x offset (in the trunks frame of reference) based on the PID controllers
    WalkResponse stabilized_response{ response };

    tf2::Transform trunk_offset;
    trunk_offset.setOrigin(tf2::Vector3(fused_pitch_correction, fused_roll_correction, 0.0));
    trunk_offset.setRotation(tf2::Quaternion::getIdentity());

    // apply the trunk offset to the trunk pose
    stabilized_response.support_foot_to_trunk = trunk_offset * response.support_foot_to_trunk;

    return stabilized_response;
  }
}  // namespace b_splines_walk
