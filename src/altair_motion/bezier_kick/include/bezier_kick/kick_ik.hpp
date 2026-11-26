#ifndef BEZIER_KICK_INCLUDE_BEZIER_KICK_KICK_IK_H_
#define BEZIER_KICK_INCLUDE_BEZIER_KICK_KICK_IK_H_

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <Eigen/Geometry>
#include <bezier_kick/kick_utils.hpp>
#include <altair_splines/abstract_ik.hpp>

namespace bezier_kick {

class KickIK : public altair_splines::AbstractIK<KickPositions> {
 public:
  KickIK() = default;
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  altair_splines::JointGoals calculate(const KickPositions &positions) override;
  void reset() override;

 private:
  moveit::core::RobotStatePtr goal_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit::core::JointModelGroup *legs_joints_group_;
  moveit::core::JointModelGroup *left_leg_joints_group_;
  moveit::core::JointModelGroup *right_leg_joints_group_;
};
}  // namespace bezier_kick

#endif  // BEZIER_KICK_INCLUDE_BEZIER_KICK_KICK_IK_H_