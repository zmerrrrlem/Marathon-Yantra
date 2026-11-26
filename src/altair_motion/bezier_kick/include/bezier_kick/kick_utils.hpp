#ifndef BEZIER_KICK_INCLUDE_BEZIER_KICK_KICK_UTILS_H_
#define BEZIER_KICK_INCLUDE_BEZIER_KICK_KICK_UTILS_H_

#include <eigen3/Eigen/Geometry>

namespace bezier_kick {

  struct KickPositions {
    bool is_left_kick = true;
    Eigen::Isometry3d trunk_pose;
    Eigen::Isometry3d flying_foot_pose;
    bool cop_support_point = false;
    double engine_time;
  };

  struct KickGoals {
    Eigen::Vector3d ball_position;
    Eigen::Quaterniond kick_direction;
    double kick_speed = 0;
    Eigen::Isometry3d trunk_to_base_footprint;
  };

  enum KickPhase
  {
    Kick_Start_Shift = 0,
    Kick_Lift = 1,
    Kick_Swing = 2,
    Kick_Retract = 3,
    Kick_Landing = 4,
    Kick_Finish_Shift = 5,
    Kick_Finished = 6,
  };

}  // namespace bezier_kick

#endif  // BEZIER_KICK_INCLUDE_BEZIER_KICK_KICK_UTILS_H_