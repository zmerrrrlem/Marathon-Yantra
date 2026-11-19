#include "cubic_walk/walk_ik.hpp"
#include <cmath>
namespace cubic_walk {

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
    // call IK two times, since we have two legs
    bool success;

    // Save previous joint positions so we can restore them if IK fails
    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> prev_joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, prev_joint_goals);

    // we have to do this otherwise there is an error
    goal_state_->updateLinkTransforms();

    success = goal_state_->setFromIK(left_leg_joints_group_, left_foot_goal_msg, config_.timeout, moveit::core::GroupStateValidityCallbackFn());
    goal_state_->updateLinkTransforms();

    success &= goal_state_->setFromIK(right_leg_joints_group_, right_foot_goal_msg, config_.timeout, moveit::core::GroupStateValidityCallbackFn());

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "IK failed with no solution found, holding last joint goals");
      goal_state_->setJointGroupPositions(legs_joints_group_, prev_joint_goals);
    }

    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

    /* construct result object */
    cubic_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;

    return result;
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
