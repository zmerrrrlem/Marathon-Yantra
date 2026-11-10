#define M_TAU M_PI * 2

#include "quintic_walk/quintic_walk.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

constexpr double DEG_TO_RAD = M_PI / 180.0;

namespace quintic_walk
{

void QuinticWalk::updateParams()
{
  config_ = param_listener_->get_params();
  // Pass params to other components
  ik_->setConfig(config_.node.ik);
  walk_engine_->setConfig(config_.engine);
  // Phase reset can only work if one phase resetting method is active and this might have changed due to parameter
  // changes, but they are in a separate struct, so we have to check them manually
  walk_engine_->setPhaseRest(config_.node.phase_reset.effort.active || config_.node.phase_reset.foot_pressure.active ||
                             config_.node.phase_reset.imu.active);
  walk_engine_->setPauseDuration(config_.node.stability_stop.pause_duration);
}

void QuinticWalk::publish_debug()
{
  visualizer_->publishDebug(current_response_, current_stabilized_response_, ik_->getLastLeftFootPose(),
                            ik_->getLastRightFootPose(), config_.node.rviz_visualization);
}

void QuinticWalk::reset()
{
  walk_engine_->reset();
  stabilizer_->reset();
}

altair_msgs::msg::JointCommand QuinticWalk::initial_position(double dt)
{
  reset();
  current_response_ = walk_engine_->update(dt);

  current_response_.current_fused_roll = current_trunk_fused_roll_;
  current_response_.current_fused_pitch = current_trunk_fused_pitch_;

  altair_msgs::msg::JointCommand command;
  motor_goals_ = ik_->calculate(current_response_);

  command.header.stamp = node_->get_clock()->now();
  /*
   * Since our JointGoals type is a vector of strings
   *  combined with a vector of numbers (motor name -> target position)
   *  and altair_msgs::msg::JointCommand needs both vectors as well,
   *  we can just assign them
   */
  command.joint_names = motor_goals_.first;
  command.positions = motor_goals_.second;

  /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
  std::vector<double> vels(motor_goals_.first.size(), -1.0);
  std::vector<double> accs(motor_goals_.first.size(), -1.0);
  std::vector<double> pwms(motor_goals_.first.size(), -1.0);
  command.velocities = vels;
  command.accelerations = accs;
  command.max_currents = pwms;
  return command;
}

altair_msgs::msg::JointCommand QuinticWalk::step(double dt)
{
  WalkRequest request(current_request_);

  // update walk engine response
  if (got_new_goals_)
  {
    got_new_goals_ = false;
    walk_engine_->setGoals(request);
  }
  checkPhaseRestAndReset();
  current_response_ = walk_engine_->update(dt);

  // only calculate joint goals from this if the engine is not idle
  current_response_.current_fused_roll = current_trunk_fused_roll_;
  current_response_.current_fused_pitch = current_trunk_fused_pitch_;

  altair_msgs::msg::JointCommand command;
  if (walk_engine_->getState() != WalkState::IDLE)
  {
    // get stabilized goals from stabilizer
    current_stabilized_response_ =
        stabilizer_->stabilize(current_response_, rclcpp::Duration::from_nanoseconds(1e9 * dt));

    // compute motor goals from IK
    motor_goals_ = ik_->calculate(current_stabilized_response_);

    command.header.stamp = node_->get_clock()->now();
    /*
     * Since our JointGoals type is a vector of strings
     *  combined with a vector of numbers (motor name -> target position)
     *  and altair_msgs::msg::JointCommand needs both vectors as well,
     *  we can just assign them
     */
    command.joint_names = motor_goals_.first;
    command.positions = motor_goals_.second;

    /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
    std::vector<double> vels(motor_goals_.first.size(), -1.0);
    std::vector<double> accs(motor_goals_.first.size(), -1.0);
    std::vector<double> pwms(motor_goals_.first.size(), -1.0);
    command.velocities = vels;
    command.accelerations = accs;
    command.max_currents = pwms;
  }

  return command;
}

double QuinticWalk::getTimeDelta()
{
  // compute time delta depended if we are currently in simulation or reality
  double dt;
  double current_ros_time = node_->get_clock()->now().seconds();
  dt = current_ros_time - last_ros_update_time_;
  last_ros_update_time_ = current_ros_time;

  // time is wrong when we run it for the first time
  if (first_run_)
  {
    first_run_ = false;
    dt = 0.0001;
  }
  return dt;
}

std::array<double, 4> QuinticWalk::get_step_from_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // We have to compute by dividing by step frequency which is a double step
  // factor 2 since the order distance is only for a single step, not double step
  double factor = (1.0 / (walk_engine_->getFreq())) / 2.0;
  // the sidewards movement only does one step per double step, since the second foot only goes back to the initial
  // pose therefore we need to multiply it by 2 furthermore, the engine does not really reach the correct goal
  // speed, dependent on the parameters
  std::array<double, 4> step = { msg->linear.x * factor * config_.node.x_speed_multiplier + config_.node.x_bias,
                                 msg->linear.y * factor * 2 * config_.node.y_speed_multiplier + config_.node.y_bias,
                                 msg->linear.z * factor,
                                 msg->angular.z * factor * config_.node.yaw_speed_multiplier + config_.node.yaw_bias };

  std::array<double, 4> clamped_step = {
    // the orders should not extend beyond a maximal step size
    std::clamp(step[0], -config_.node.max_step_x, config_.node.max_step_x),
    std::clamp(step[1], -config_.node.max_step_y, config_.node.max_step_y),
    std::clamp(step[2], -config_.node.max_step_z, config_.node.max_step_z),
    std::max(std::min(step[3], config_.node.max_step_angular), config_.node.max_step_angular * -1)
  };

  // translational orders (x+y) should not exceed combined limit. scale if necessary
  if (config_.node.max_step_xy != 0)
  {
    double scaling_factor = sqrt(pow(clamped_step[0], 2) + pow(clamped_step[1], 2)) / config_.node.max_step_xy;
    for (int i = 0; i < 2; i++)
    {
      clamped_step[i] = clamped_step[i] / std::max(scaling_factor, 1.0);
    }
  }

  // warn user that speed was limited
  if (step != clamped_step)
  {
    RCLCPP_WARN(node_->get_logger(),
                "Commanded step 'x: %.2f y: %.2f z: %.2f angular: %.2f' exceeded limits, clamped to 'x: %.2f y: %.2f "
                "z: "
                "%.2f angular: %.2f'",
                step[0], step[1], step[2], step[3], clamped_step[0], clamped_step[1], clamped_step[2], clamped_step[3]);
  }

  return clamped_step;
}

void QuinticWalk::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<std::string> names = msg->name;

  // compute the effort that is currently on the flying leg to check if it has ground contact
  // only if we have the necessary data
  if (msg->effort.size() == msg->name.size())
  {
    double effort_sum = 0;
    const std::vector<std::string>& fly_joint_names =
        (walk_engine_->isLeftSupport()) ? ik_->getRightLegJointNames() : ik_->getLeftLegJointNames();
    for (size_t i = 0; i < names.size(); i++)
    {
      // add effort on this joint to sum, if it is part of the flying leg
      if (std::find(fly_joint_names.begin(), fly_joint_names.end(), names[i]) != fly_joint_names.end())
      {
        effort_sum = effort_sum + abs(msg->effort[i]);
      }
    }
    current_fly_effort_ = effort_sum;
  }
  else
  {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "Joint states have no effort information. Phase reset based on this will not work.");
  }
}

void QuinticWalk::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double roll, pitch, yaw;

  // the incoming geometry_msgs::msg::Quaternion is transformed to a tf2::Quaternion
  tf2::Quaternion quat;
  tf2::convert(msg->orientation, quat);
  // the tf2::Quaternion has a method to access roll pitch and yaw

  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  Eigen::Quaterniond imu_orientation_eigen = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
  rot_conv::FusedAngles imu_fused = rot_conv::FusedFromQuat(imu_orientation_eigen);

  current_trunk_fused_pitch_ = imu_fused.fusedPitch;
  current_trunk_fused_roll_ = imu_fused.fusedRoll;

  // get angular velocities
  double roll_vel = msg->angular_velocity.x;
  double pitch_vel = msg->angular_velocity.y;

  // Calculate ema (exponential moving average) of y (sideways) acceleration in a time-independet manner
  if (last_imu_measurement_time_)
  {
    auto time_delta = rclcpp::Time(msg->header.stamp) - last_imu_measurement_time_.value();
    double exponent_in_s = -time_delta.nanoseconds() / (config_.node.imu_y_acc_tau * 1e9);
    double alpha = 1 - std::exp(exponent_in_s);

    imu_y_acc = msg->linear_acceleration.y * alpha + imu_y_acc * (1 - alpha);
  }
  else
  {
    imu_y_acc = msg->linear_acceleration.y;
  }

  if (config_.node.stability_stop.imu.active)
  {
    // compute the pitch offset to the currently wanted pitch of the engine
    double wanted_pitch = walk_engine_->getWantedTrunkPitch();

    // Get the sub struct of the imu stability stop config
    auto params = config_.node.stability_stop.imu;

    // Check if we have to stop the walk
    double pitch_delta = pitch - wanted_pitch;
    if (abs(roll) > params.roll.threshold or abs(pitch_delta) > params.pitch.threshold or
        abs(pitch_vel) > params.pitch.vel_threshold or abs(roll_vel) > params.roll.vel_threshold)
    {
      walk_engine_->requestPause();
      if (abs(roll) > params.roll.threshold)
      {
        RCLCPP_WARN(node_->get_logger(), "imu roll angle stop");
      }
      else if (abs(pitch_delta) > params.pitch.threshold)
      {
        RCLCPP_WARN(node_->get_logger(), "imu pitch angle stop");
      }
      else if (abs(pitch_vel) > params.pitch.vel_threshold)
      {
        RCLCPP_WARN(node_->get_logger(), "imu pitch vel stop");
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "imu roll vel stop");
      }
    }
  }
  // Store the timestamp
  last_imu_measurement_time_ = msg->header.stamp;
}

void QuinticWalk::stepCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_request_.linear_orders = { msg->linear.x, msg->linear.y, msg->linear.z };
  current_request_.angular_z = msg->angular.z;
  current_request_.single_step = true;
  current_request_.stop_walk = true;
  got_new_goals_ = true;
}

void QuinticWalk::cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  got_new_goals_ = true;
  current_request_.single_step = false;

  // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
  // other axis.

  // the engine expects orders in [m] not [m/s]
  auto step = get_step_from_vel(msg);
  current_request_.linear_orders = { step[0], step[1], step[2] };
  current_request_.angular_z = step[3];

  // special command to completely stop the walking
  current_request_.stop_walk = msg->angular.x != 0;
}

void QuinticWalk::checkPhaseRestAndReset()
{
  /**
   * This method checks if the foot made contact to the ground and ends the step earlier by resetting the phase
   ("phase reset") or lets the robot rest until it makes ground contact ("phase rest").
   */
  double phase = walk_engine_->getPhase();
  // Check if we are in the correct point of the phase to do a phase reset
  // Phase has to be far enough (almost at end of step) so that the foot has already lifted from the ground
  // otherwise we will always do phase reset in the beginning of the step
  double min_phase_single_step = config_.node.phase_reset.min_phase / 2;
  if ((phase > min_phase_single_step and phase < 0.5) or (phase > 0.5 + min_phase_single_step))
  {
    // Check if one of our phase reset conditions is met
    if ((config_.node.phase_reset.foot_pressure.active and
         current_fly_pressure_ > config_.node.phase_reset.foot_pressure.ground_min_pressure) or
        (config_.node.phase_reset.effort.active and
         current_fly_effort_ > config_.node.phase_reset.effort.joint_min_effort) or
        (config_.node.phase_reset.imu.active and
         std::abs(imu_y_acc) < config_.node.phase_reset.imu.y_acceleration_threshold))
    {
      // Manually end the step
      // This may end a step earlier than the engine would do it
      // But it can also lengthen the step, as the engine will not end the step by itself if a phase reset is active
      walk_engine_->endStep();
    }
  }
}

void QuinticWalk::robotStateCb(const altair_msgs::msg::RobotControlState::SharedPtr msg)
{
  robot_state_ = msg->state;
}

void QuinticWalk::kickCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  walk_engine_->requestKick(msg->data);
}

nav_msgs::msg::Odometry QuinticWalk::getOdometry()
{
  // odometry to trunk is transform to support foot * transform from support to trunk
  tf2::Transform support_foot_tf;
  if (walk_engine_->isLeftSupport())
  {
    support_foot_tf = walk_engine_->getLeft();
  }
  else
  {
    support_foot_tf = walk_engine_->getRight();
  }

  tf2::Transform odom_to_trunk = support_foot_tf * current_response_.support_foot_to_trunk;
  tf2::Vector3 pos = odom_to_trunk.getOrigin();
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::Quaternion quat_normalized = odom_to_trunk.getRotation().normalize();
  quat_msg.x = quat_normalized.x();
  quat_msg.y = quat_normalized.y();
  quat_msg.z = quat_normalized.z();
  quat_msg.w = quat_normalized.w();

  rclcpp::Time current_time = node_->get_clock()->now();

  // send the odometry as message
  odom_msg_.header.stamp = current_time;
  odom_msg_.header.frame_id = config_.node.tf.odom_frame;
  odom_msg_.child_frame_id = config_.node.tf.base_link_frame;
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];
  odom_msg_.pose.pose.orientation = quat_msg;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = current_request_.linear_orders[0] * walk_engine_->getFreq() * 2 / config_.node.x_speed_multiplier;
  twist.linear.y = current_request_.linear_orders[1] * walk_engine_->getFreq() / config_.node.y_speed_multiplier;
  twist.linear.z = current_request_.linear_orders[2] * walk_engine_->getFreq() * 2;
  twist.angular.z = current_request_.angular_z * walk_engine_->getFreq() * 2 / config_.node.yaw_speed_multiplier;

  odom_msg_.twist.twist = twist;

  // update the tf odom in rviz ============= ONLY FOR DEBUG, PLEASE REMOVE THIS IN THE FUTURE AND CREATE ODOMOTRY_FUSER
  // PACKAGE
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = current_time;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = pos[0];
  transform.transform.translation.y = pos[1];
  transform.transform.translation.z = pos[2];
  transform.transform.rotation = quat_msg;

  // tf_broadcaster->sendTransform(transform);

  return odom_msg_;
}

altair_msgs::msg::Stabilizer QuinticWalk::getStabilizerData()
{
  return stabilizer_->getStabilizerData();
}

void QuinticWalk::initializeEngine()
{
  walk_engine_->reset();
}

WalkEngine* QuinticWalk::getEngine()
{
  return walk_engine_.get();
}

WalkIK* QuinticWalk::getIk()
{
  return ik_.get();
}

double QuinticWalk::getTimerFreq()
{
  return config_.node.engine_freq;
}

void QuinticWalk::move_smoother(double& buffer, double current, double target)
{
  // Arguments: current: reference to current servo position, target: target servo position
  // Optional steps parameter to control smoothing rate (higher = smoother but slower)
  if (mot_cont_ < 1)
  {
    buffer = target;
  }
  else
  {
    buffer += (target - current) / mot_cont_;
  }
}

void QuinticWalk::applyJointCommandToResult(const altair_msgs::msg::JointCommand& command,
                                            const std::map<std::string, robotis_framework::Dynamixel*>& dxls,
                                            std::size_t max_joints)
{
  const auto available = std::min(command.joint_names.size(), command.positions.size());
  const auto joint_count = std::min({ available, max_joints, static_cast<std::size_t>(12) });

  if (joint_count == 0)
    return;

  for (std::size_t idx = 0; idx < joint_count; ++idx)
    setJointTarget(command.joint_names[idx], command.positions[idx], dxls);
}

void QuinticWalk::setJointTarget(const std::string& joint_name, double target_position,
                                 const std::map<std::string, robotis_framework::Dynamixel*>& dxls)
{
  // Log the target position
  // RCLCPP_INFO(node_->get_logger(), "Setting target position for joint '%s' to %f", joint_name.c_str(),
  // target_position);
  if (result_.find(joint_name) != result_.end())
  {
    result_[joint_name]->goal_position_ = target_position;
  }
}

}  // namespace quintic_walk
