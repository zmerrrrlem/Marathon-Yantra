#include "quintic_walk/walk_ik.hpp"

#include <cmath>

namespace quintic_walk {

  namespace {
    tf2::Vector3 toTf(const Eigen::Vector3d& v) { return tf2::Vector3(v.x(), v.y(), v.z()); }

    bool isFinitePose(const geometry_msgs::msg::Pose& p) {
      return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
        std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
        std::isfinite(p.orientation.w);
    }
  }  // namespace

  WalkIK::WalkIK(rclcpp_lifecycle::LifecycleNode::SharedPtr node, walking::Params::Node::Ik config) : node_(node), config_(config) {}

  void WalkIK::init(moveit::core::RobotModelPtr kinematic_model) {
    legs_joints_group_ = kinematic_model->getJointModelGroup("Legs");
    left_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
    right_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");

    if (!legs_joints_group_ || !left_leg_joints_group_ || !right_leg_joints_group_) {
      RCLCPP_ERROR(node_->get_logger(),
        "SRDF is missing a joint group (Legs=%s LeftLeg=%s RightLeg=%s). IK is disabled and no leg goals will be sent.",
        legs_joints_group_ ? "ok" : "MISSING", left_leg_joints_group_ ? "ok" : "MISSING",
        right_leg_joints_group_ ? "ok" : "MISSING");
    }

    goal_state_.reset(new moveit::core::RobotState(kinematic_model));
    goal_state_->setToDefaultValues();
    goal_state_->updateLinkTransforms();

    left_leg_geometry_ = measureLeg(left_leg_joints_group_);
    right_leg_geometry_ = measureLeg(right_leg_joints_group_);

    if (config_.reset) {
      reset();
    }
  }

  WalkIK::LegGeometry WalkIK::measureLeg(const moveit::core::JointModelGroup* leg_group) {
    LegGeometry geometry;
    if (!leg_group) return geometry;

    // Expected chain (root -> tip): hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll.
    const auto& joints = leg_group->getActiveJointModels();
    if (joints.size() != 6) {
      RCLCPP_WARN(node_->get_logger(),
        "Leg group '%s' has %zu active joints (expected 6). Workspace clamping disabled for this leg.",
        leg_group->getName().c_str(), joints.size());
      return geometry;
    }

    // Measure in the zero configuration, independent of the reset pose.
    moveit::core::RobotState zero_state(goal_state_->getRobotModel());
    zero_state.setToDefaultValues();
    std::vector<double> zeros(leg_group->getVariableCount(), 0.0);
    zero_state.setJointGroupPositions(leg_group, zeros);
    zero_state.updateLinkTransforms();

    const auto* hip_link = joints[2]->getChildLinkModel();
    const auto* knee_link = joints[3]->getChildLinkModel();
    const auto* ankle_link = joints[4]->getChildLinkModel();
    const auto* tip_link = joints[5]->getChildLinkModel();

    const Eigen::Isometry3d& hip = zero_state.getGlobalLinkTransform(hip_link);
    const Eigen::Isometry3d& knee = zero_state.getGlobalLinkTransform(knee_link);
    const Eigen::Isometry3d& ankle = zero_state.getGlobalLinkTransform(ankle_link);
    const Eigen::Isometry3d& tip = zero_state.getGlobalLinkTransform(tip_link);

    double thigh = (knee.translation() - hip.translation()).norm();
    double shank = (ankle.translation() - knee.translation()).norm();
    if (thigh < 0.01 || shank < 0.01) {
      RCLCPP_WARN(node_->get_logger(),
        "Leg group '%s': implausible thigh/shank length (%.3f / %.3f m). Workspace clamping disabled for this leg.",
        leg_group->getName().c_str(), thigh, shank);
      return geometry;
    }

    geometry.hip_in_root = toTf(hip.translation());
    geometry.tip_to_ankle = toTf(tip.inverse() * ankle.translation());
    geometry.max_reach = thigh + shank;
    geometry.valid = true;

    RCLCPP_INFO(node_->get_logger(),
      "Leg '%s': hip=%s knee=%s ankle=%s tip=%s, thigh %.4f m + shank %.4f m = reach %.4f m",
      leg_group->getName().c_str(), joints[2]->getName().c_str(), joints[3]->getName().c_str(),
      joints[4]->getName().c_str(), tip_link->getName().c_str(), thigh, shank, geometry.max_reach);
    return geometry;
  }

  geometry_msgs::msg::Pose WalkIK::clampToReach(const geometry_msgs::msg::Pose& goal, const LegGeometry& leg,
    bool& clamped) const {
    clamped = false;
    if (!leg.valid) return goal;

    tf2::Transform goal_tf;
    tf2::fromMsg(goal, goal_tf);

    // Where the ankle-pitch joint would be if the tip reached the goal
    tf2::Vector3 ankle = goal_tf * leg.tip_to_ankle;
    tf2::Vector3 hip_to_ankle = ankle - leg.hip_in_root;
    double distance = hip_to_ankle.length();
    double limit = config_.reach_margin * leg.max_reach;
    if (distance <= limit || distance < 1e-9) return goal;

    // Pull the ankle back onto the reachable sphere along the hip->ankle line, keep the foot orientation
    tf2::Vector3 shift = hip_to_ankle * (limit / distance) - hip_to_ankle;
    geometry_msgs::msg::Pose result = goal;
    result.position.x += shift.x();
    result.position.y += shift.y();
    result.position.z += shift.z();
    clamped = true;
    return result;
  }

  bool WalkIK::solveLeg(const moveit::core::JointModelGroup* leg_group, const geometry_msgs::msg::Pose& goal) {
    std::vector<double> previous;
    goal_state_->copyJointGroupPositions(leg_group, previous);

    bool success = isFinitePose(goal) &&
      goal_state_->setFromIK(leg_group, goal, config_.timeout, moveit::core::GroupStateValidityCallbackFn());

    if (success) {
      std::vector<double> solution;
      goal_state_->copyJointGroupPositions(leg_group, solution);
      for (double q : solution) {
        if (!std::isfinite(q)) {
          success = false;
          break;
        }
      }
    }

    if (!success && config_.hold_last_on_failure) {
      goal_state_->setJointGroupPositions(leg_group, previous);
    }
    goal_state_->updateLinkTransforms();
    return success;
  }

  quintic_splines::JointGoals WalkIK::calculate(const WalkResponse& ik_goals) {
    if (!legs_joints_group_ || !left_leg_joints_group_ || !right_leg_joints_group_) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "IK skipped: leg joint groups missing from the robot model");
      return {};
    }

    // change goals from support foot based coordinate system to trunk based coordinate system
    tf2::Transform trunk_to_support_foot_goal = ik_goals.support_foot_to_trunk.inverse();
    tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * ik_goals.support_foot_to_flying_foot;

    // make pose msg for calling IK
    geometry_msgs::msg::Pose left_foot_goal_msg;
    geometry_msgs::msg::Pose right_foot_goal_msg;

    // decide which foot is which
    if (ik_goals.is_left_support_foot) {
      tf2::toMsg(trunk_to_support_foot_goal, left_foot_goal_msg);
      tf2::toMsg(trunk_to_flying_foot_goal, right_foot_goal_msg);
    }
    else {
      tf2::toMsg(trunk_to_support_foot_goal, right_foot_goal_msg);
      tf2::toMsg(trunk_to_flying_foot_goal, left_foot_goal_msg);
    }

    // Keep goals inside the reachable workspace so the solver is not asked for the impossible
    if (config_.clamp_to_workspace) {
      bool left_clamped = false;
      bool right_clamped = false;
      left_foot_goal_msg = clampToReach(left_foot_goal_msg, left_leg_geometry_, left_clamped);
      right_foot_goal_msg = clampToReach(right_foot_goal_msg, right_leg_geometry_, right_clamped);
      if (left_clamped || right_clamped) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Foot goal beyond leg reach, clamped (left=%d right=%d). Check trunk_height / step size.",
          left_clamped, right_clamped);
      }
    }

    // we have to do this otherwise there is an error
    goal_state_->updateLinkTransforms();

    // Solve each leg independently so one failure does not corrupt the other
    bool left_success = solveLeg(left_leg_joints_group_, left_foot_goal_msg);
    bool right_success = solveLeg(right_leg_joints_group_, right_foot_goal_msg);

    if (left_success && right_success) {
      consecutive_failures_ = 0;
    }
    else {
      consecutive_failures_++;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "IK failed (left=%s right=%s), %d consecutive cycle(s); %s",
        left_success ? "ok" : "FAIL", right_success ? "ok" : "FAIL", consecutive_failures_,
        config_.hold_last_on_failure ? "holding last valid joint positions" : "sending solver output as-is");
    }

    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

    /* construct result object */
    quintic_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  }

  void WalkIK::reset() {
    // we have to set some good initial position in the goal state, since we are using a gradient
    // based method. Otherwise, the first step will be not correct
    std::vector<std::string> names_vec = { "l_hip_pitch", "l_knee", "l_ank_pitch", "r_hip_pitch", "r_knee", "r_ank_pitch" };
    std::vector<double> pos_vec = { 0.7, 1.0, -0.4, -0.7, -1.0, 0.4 };
    for (size_t i = 0; i < names_vec.size(); i++) {
      if (!goal_state_->getRobotModel()->hasJointModel(names_vec[i])) {
        RCLCPP_WARN(node_->get_logger(), "IK reset: joint '%s' not in robot model, skipped", names_vec[i].c_str());
        continue;
      }
      // besides its name, this method only changes a single joint position...
      goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
    }
    goal_state_->updateLinkTransforms();
    consecutive_failures_ = 0;
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

}  // namespace quintic_walk
