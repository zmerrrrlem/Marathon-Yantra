#include "quintic_walk/walk_ik.hpp"
namespace quintic_walk {

  WalkIK::WalkIK(rclcpp_lifecycle::LifecycleNode::SharedPtr node, walking::Params::Node::Ik config) : node_(node), config_(config) {}

  void WalkIK::init(moveit::core::RobotModelPtr kinematic_model) {
    legs_joints_group_ = kinematic_model->getJointModelGroup("Legs");
    left_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
    right_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");

    goal_state_.reset(new moveit::core::RobotState(kinematic_model));
    goal_state_->setToDefaultValues();
    if (config_.reset) {
      reset();
    }
  }

  quintic_splines::JointGoals WalkIK::calculate(const WalkResponse& ik_goals) {
    // auto start = std::chrono::high_resolution_clock::now();
    // change goals from support foot based coordinate system to trunk based coordinate system
    // Print out all data in ik_goals for debugging
    // RCLCPP_INFO(node_->get_logger(), "WalkIK::calculate - Printing ik_goals data:");
    // RCLCPP_INFO(node_->get_logger(), "Is left support foot: %s", ik_goals.is_left_support_foot ? "true" : "false");

    // Print support_foot_to_trunk transform
    // tf2::Vector3 support_to_trunk_translation = ik_goals.support_foot_to_trunk.getOrigin();
    // tf2::Quaternion support_to_trunk_rotation = ik_goals.support_foot_to_trunk.getRotation();
    // RCLCPP_INFO(node_->get_logger(), "Support foot to trunk translation: x=%f, y=%f, z=%f",
    //       support_to_trunk_translation.x(), support_to_trunk_translation.y(), support_to_trunk_translation.z());
    // RCLCPP_INFO(node_->get_logger(), "Support foot to trunk rotation: x=%f, y=%f, z=%f, w=%f",
    //       support_to_trunk_rotation.x(), support_to_trunk_rotation.y(), 
    //       support_to_trunk_rotation.z(), support_to_trunk_rotation.w());

    // Print support_foot_to_flying_foot transform
    // tf2::Vector3 support_to_flying_translation = ik_goals.support_foot_to_flying_foot.getOrigin();
    // tf2::Quaternion support_to_flying_rotation = ik_goals.support_foot_to_flying_foot.getRotation();
    // RCLCPP_INFO(node_->get_logger(), "Support foot to flying foot translation: x=%f, y=%f, z=%f",
    //       support_to_flying_translation.x(), support_to_flying_translation.y(), support_to_flying_translation.z());
    // RCLCPP_INFO(node_->get_logger(), "Support foot to flying foot rotation: x=%f, y=%f, z=%f, w=%f",
    //       support_to_flying_rotation.x(), support_to_flying_rotation.y(), 
    //       support_to_flying_rotation.z(), support_to_flying_rotation.w());

        
    tf2::Transform trunk_to_support_foot_goal = ik_goals.support_foot_to_trunk.inverse();
    tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * ik_goals.support_foot_to_flying_foot;
    
    // Print trunk_to_support_foot_goal transform
    // tf2::Vector3 trunk_to_support_translation = trunk_to_support_foot_goal.getOrigin();
    // tf2::Quaternion trunk_to_support_rotation = trunk_to_support_foot_goal.getRotation();
    // RCLCPP_INFO(node_->get_logger(), "Trunk to support foot translation: x=%f, y=%f, z=%f",
    //     trunk_to_support_translation.x(), trunk_to_support_translation.y(), trunk_to_support_translation.z());
    // RCLCPP_INFO(node_->get_logger(), "Trunk to support foot rotation: x=%f, y=%f, z=%f, w=%f",
    //     trunk_to_support_rotation.x(), trunk_to_support_rotation.y(), 
    //     trunk_to_support_rotation.z(), trunk_to_support_rotation.w());
        
    // Print trunk_to_flying_foot_goal transform
    // tf2::Vector3 trunk_to_flying_translation = trunk_to_flying_foot_goal.getOrigin();
    // tf2::Quaternion trunk_to_flying_rotation = trunk_to_flying_foot_goal.getRotation();
    // RCLCPP_INFO(node_->get_logger(), "Trunk to flying foot translation: x=%f, y=%f, z=%f",
    //     trunk_to_flying_translation.x(), trunk_to_flying_translation.y(), trunk_to_flying_translation.z());
    // RCLCPP_INFO(node_->get_logger(), "Trunk to flying foot rotation: x=%f, y=%f, z=%f, w=%f",
    //     trunk_to_flying_rotation.x(), trunk_to_flying_rotation.y(), 
    //     trunk_to_flying_rotation.z(), trunk_to_flying_rotation.w());

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

    // we have to do this otherwise there is an error
    goal_state_->updateLinkTransforms();

    success = goal_state_->setFromIK(left_leg_joints_group_, left_foot_goal_msg, config_.timeout, moveit::core::GroupStateValidityCallbackFn());
    goal_state_->updateLinkTransforms();

    success &= goal_state_->setFromIK(right_leg_joints_group_, right_foot_goal_msg, config_.timeout, moveit::core::GroupStateValidityCallbackFn());

    // if (!success) {
    //   RCLCPP_ERROR(node_->get_logger(), "IK failed with no solution found");
    // }
    // else {
    //   RCLCPP_INFO(node_->get_logger(), "IK success");
    // }

    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);

    /* construct result object */
    quintic_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;

    // auto end = std::chrono::high_resolution_clock::now();
    
    // Calculate duration
    // std::chrono::duration<double, std::milli> duration = end - start;
    
    // Print execution time
    // RCLCPP_INFO_STREAM(node_->get_logger(), "IK solving took " << duration.count() << " ms");

    return result;
  }

  void WalkIK::reset() {
    // we have to set some good initial position in the goal state, since we are using a gradient
    // based method. Otherwise, the first step will be not correct
    std::vector<std::string> names_vec = { "l_hip_pitch", "l_knee", "l_ank_pitch", "r_hip_pitch", "r_knee", "r_ank_pitch" };
    std::vector<double> pos_vec = { 0.7, 1.0, -0.4, -0.7, -1.0, 0.4 };
    for (size_t i = 0; i < names_vec.size(); i++) {
      // besides its name, this method only changes a single joint position...
      goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
    }
  }

  void WalkIK::setConfig(walking::Params::Node::Ik config) { config_ = config; }

  const std::vector<std::string>& WalkIK::getLeftLegJointNames() { return left_leg_joints_group_->getJointModelNames(); }

  const std::vector<std::string>& WalkIK::getRightLegJointNames() { return right_leg_joints_group_->getJointModelNames(); }

  moveit::core::RobotStatePtr WalkIK::get_goal_state() { return goal_state_; }

}  // namespace quintic_walk
