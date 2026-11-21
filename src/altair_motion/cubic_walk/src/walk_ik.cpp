#include "cubic_walk/walk_ik.hpp"
#include <cmath>
namespace cubic_walk {

  // Yantra leg max reach to sole frame (~0.323 m). Poses beyond this are clamped
  // to avoid IK failures when the walk engine commands unreachable foot positions.
  static constexpr double MAX_LEG_REACH = 0.30;  // conservative margin below 0.323 m

  static constexpr double MAX_FOOT_X = 0.25;   // max forward/backward foot offset
  static constexpr double MAX_FOOT_Y = 0.10;   // max sideways foot offset
  static constexpr double MAX_FOOT_Z = 0.35;   // max foot height above ground
  static constexpr double MIN_FOOT_Z = 0.05;   // min foot height
  static constexpr double MAX_FOOT_ROLL = 0.2;   // max foot roll angle (rad)
  static constexpr double MAX_FOOT_PITCH = 0.2;  // max foot pitch angle (rad)

  static geometry_msgs::msg::Pose clampPoseToReach(const geometry_msgs::msg::Pose& pose) {
    geometry_msgs::msg::Pose clamped = pose;

    // Clamp position per-axis (preserves orientation better than uniform scaling)
    clamped.position.x = std::clamp(pose.position.x, -MAX_FOOT_X, MAX_FOOT_X);
    clamped.position.y = std::clamp(pose.position.y, -MAX_FOOT_Y, MAX_FOOT_Y);
    clamped.position.z = std::clamp(pose.position.z, MIN_FOOT_Z, MAX_FOOT_Z);

    // Normalize quaternion
    double norm = std::sqrt(
      clamped.orientation.x * clamped.orientation.x +
      clamped.orientation.y * clamped.orientation.y +
      clamped.orientation.z * clamped.orientation.z +
      clamped.orientation.w * clamped.orientation.w);
    if (norm > 0.0) {
      clamped.orientation.x /= norm;
      clamped.orientation.y /= norm;
      clamped.orientation.z /= norm;
      clamped.orientation.w /= norm;
    } else {
      clamped.orientation.x = 0.0;
      clamped.orientation.y = 0.0;
      clamped.orientation.z = 0.0;
      clamped.orientation.w = 1.0;
    }

    // Clamp roll and pitch of foot orientation to prevent extreme angles
    tf2::Quaternion quat;
    tf2::fromMsg(clamped.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    roll = std::clamp(roll, -MAX_FOOT_ROLL, MAX_FOOT_ROLL);
    pitch = std::clamp(pitch, -MAX_FOOT_PITCH, MAX_FOOT_PITCH);
    quat.setRPY(roll, pitch, yaw);
    clamped.orientation.x = quat.x();
    clamped.orientation.y = quat.y();
    clamped.orientation.z = quat.z();
    clamped.orientation.w = quat.w();

    return clamped;
  }

  WalkIK::WalkIK(rclcpp_lifecycle::LifecycleNode::SharedPtr node, walking::Params::Node::Ik config) : node_(node), config_(config) {}

  void WalkIK::init(moveit::core::RobotModelPtr kinematic_model) {
    legs_joints_group_ = kinematic_model->getJointModelGroup("Legs");
    left_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
    right_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");

    goal_state_.reset(new moveit::core::RobotState(kinematic_model));
    goal_state_->setToDefaultValues();
    if (config_.reset && legs_joints_group_) {
      reset();
    }
    if (!legs_joints_group_) {
      RCLCPP_WARN(node_->get_logger(), "Legs joint model group not found in URDF — IK will not work");
    }
  }

  cubic_splines::JointGoals WalkIK::calculate(const WalkResponse& ik_goals) {
    if (!legs_joints_group_) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Cannot calculate IK: Legs joint model group is null");
      return {{}, {}};
    }

    // Get current trunk pose in root (dummy_root) frame
    // This is needed because setFromIK() expects poses in root frame,
    // but the walk engine computes poses relative to trunk.
    tf2::Transform root_to_trunk;
    try {
      tf2::fromMsg(goal_state_->getFrameTransform("base_link"), root_to_trunk);
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Could not get trunk transform from robot state, using identity");
      root_to_trunk.setIdentity();
    }

    // change goals from support foot based coordinate system to trunk based coordinate system
    tf2::Transform trunk_to_support_foot_goal = ik_goals.support_foot_to_trunk.inverse();
    tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * ik_goals.support_foot_to_flying_foot;

    // Convert foot poses from trunk frame to root frame
    // IK expects poses in root frame (dummy_root), but walk engine gives trunk-relative poses
    tf2::Transform root_to_support_foot = root_to_trunk * trunk_to_support_foot_goal;
    tf2::Transform root_to_flying_foot = root_to_trunk * trunk_to_flying_foot_goal;

    // make pose msg for calling IK
    geometry_msgs::msg::Pose left_foot_goal_msg;
    geometry_msgs::msg::Pose right_foot_goal_msg;

    // decide which foot is which
    if (ik_goals.is_left_support_foot) {
      tf2::toMsg(root_to_support_foot, left_foot_goal_msg);
      tf2::toMsg(root_to_flying_foot, right_foot_goal_msg);
    }
    else {
      tf2::toMsg(root_to_support_foot, right_foot_goal_msg);
      tf2::toMsg(root_to_flying_foot, left_foot_goal_msg);
    }

    // Clamp foot poses to Yantra's reachable workspace to prevent IK failures
    left_foot_goal_msg = clampPoseToReach(left_foot_goal_msg);
    right_foot_goal_msg = clampPoseToReach(right_foot_goal_msg);

    // Debug logging for IK failures (first occurrence only to avoid log spam)
    static bool ik_debug_logged = false;
    if (!ik_debug_logged) {
      RCLCPP_INFO(node_->get_logger(),
        "IK debug: support foot pose = (%.3f, %.3f, %.3f), flying foot pose = (%.3f, %.3f, %.3f)",
        left_foot_goal_msg.position.x, left_foot_goal_msg.position.y, left_foot_goal_msg.position.z,
        right_foot_goal_msg.position.x, right_foot_goal_msg.position.y, right_foot_goal_msg.position.z);
      ik_debug_logged = true;
    }

    // call IK two times, since we have two legs

    // Save previous joint positions so we can restore them if IK fails
    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> prev_joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, prev_joint_goals);

    // we have to do this otherwise there is an error
    goal_state_->updateLinkTransforms();

    // Solve IK for each leg independently, so one failure doesn't freeze both legs
    bool left_success = goal_state_->setFromIK(left_leg_joints_group_, left_foot_goal_msg, config_.timeout, moveit::core::GroupStateValidityCallbackFn());
    goal_state_->updateLinkTransforms();

    bool right_success = goal_state_->setFromIK(right_leg_joints_group_, right_foot_goal_msg, config_.timeout, moveit::core::GroupStateValidityCallbackFn());
    goal_state_->updateLinkTransforms();

    if (!left_success || !right_success) {
      RCLCPP_ERROR(node_->get_logger(), "IK failed: left=%d right=%d, restoring standing pose", left_success, right_success);

      // Restore to default standing pose instead of holding last (possibly bad) goals
      if (!left_success) {
        resetLegToStanding(left_leg_joints_group_);
      }
      if (!right_success) {
        resetLegToStanding(right_leg_joints_group_);
      }
    }

    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

    /* construct result object */
    cubic_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;

    return result;
  }

  void WalkIK::resetLegToStanding(const moveit::core::JointModelGroup* leg_group) {
    if (!leg_group) return;

    auto joint_names = leg_group->getActiveJointModelNames();
    std::vector<double> standing_positions;

    for (const auto& name : joint_names) {
      // Default standing positions for Yantra leg joints
      // These match the reset() values for pitch joints, zeros for others
      double pos = 0.0;
      if (name.find("pitch_1") != std::string::npos) pos = 0.7;
      else if (name.find("pitch_3") != std::string::npos) pos = 1.0;
      else if (name.find("pitch_5") != std::string::npos) pos = -0.4;
      else if (name.find("pitch_2") != std::string::npos) pos = -0.7;
      else if (name.find("pitch_4") != std::string::npos) pos = -1.0;
      else if (name.find("pitch_6") != std::string::npos) pos = 0.4;
      standing_positions.push_back(pos);
    }

    goal_state_->setJointGroupPositions(leg_group, standing_positions);
  }

  std::vector<std::pair<std::string, double>> WalkIK::getArmSwingGoals(double phase, double amp_pitch, double amp_roll, double elbow_bend) {
    std::vector<std::pair<std::string, double>> goals;

    // Arms swing opposite to legs: left arm forward when right leg swings
    // Phase 0.0-0.5: left leg support, right leg swing -> right arm forward
    // Phase 0.5-1.0: right leg support, left leg swing -> left arm forward

    double two_pi_phase = 2.0 * M_PI * phase;

    // Left arm (swings forward when right leg is in swing, phase 0.5-1.0)
    double left_pitch = amp_pitch * std::sin(two_pi_phase + M_PI);  // opposite phase
    double left_roll = amp_roll * std::sin(two_pi_phase + M_PI);
    goals.push_back({"left_arm_pitch_3_joint", left_pitch});
    goals.push_back({"left_arm_pitch_1_joint", left_pitch});
    goals.push_back({"left_arm_roll_1_joint", left_roll});
    goals.push_back({"left_claw_joint", elbow_bend});

    // Right arm - same phase as left arm
    double right_pitch = amp_pitch * std::sin(two_pi_phase + M_PI);
    double right_roll = amp_roll * std::sin(two_pi_phase + M_PI);
    goals.push_back({"right_arm_pitch_4_joint", right_pitch});
    goals.push_back({"right_arm_pitch_2_joint", right_pitch});
    goals.push_back({"right_arm_roll_2_joint", right_roll});
    goals.push_back({"right_claw_joint", -elbow_bend});  // mirror: +1.2 rad

    return goals;
  }

  void WalkIK::reset() {
    // we have to set some good initial position in the goal state, since we are using a gradient
    // based method. Otherwise, the first step will be not correct
    std::vector<std::string> names_vec = { "left_leg_pitch_1_joint", "left_leg_pitch_3_joint", "left_leg_pitch_5_joint", "right_leg_pitch_2_joint", "right_leg_pitch_4_joint", "right_leg_pitch_6_joint" };
    std::vector<double> pos_vec = { 0.7, 1.0, -0.4, -0.7, -1.0, 0.4 };
    for (size_t i = 0; i < names_vec.size(); i++) {
      // besides its name, this method only changes a single joint position...
      const moveit::core::JointModel* jm = goal_state_->getJointModel(names_vec[i]);
      if (jm) {
        goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
      } else {
        RCLCPP_WARN(node_->get_logger(), "Joint '%s' not found in URDF, skipping reset position", names_vec[i].c_str());
      }
    }
  }

  void WalkIK::setConfig(walking::Params::Node::Ik config) { config_ = config; }

  const std::vector<std::string>& WalkIK::getLeftLegJointNames() {
    static const std::vector<std::string> empty;
    return left_leg_joints_group_ ? left_leg_joints_group_->getJointModelNames() : empty;
  }

  const std::vector<std::string>& WalkIK::getRightLegJointNames() {
    static const std::vector<std::string> empty;
    return right_leg_joints_group_ ? right_leg_joints_group_->getJointModelNames() : empty;
  }

  moveit::core::RobotStatePtr WalkIK::get_goal_state() { return goal_state_; }

}  // namespace cubic_walk
