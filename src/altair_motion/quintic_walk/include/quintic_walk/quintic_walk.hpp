#ifndef QUINTIC_WALK__QUINTIC_WALK_HPP_
#define QUINTIC_WALK__QUINTIC_WALK_HPP_

#ifdef SENSOR
#undef SENSOR
#endif

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <cmath>  // For M_PI
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "biped_interfaces/msg/phase.hpp"
#include "altair_msgs/msg/joint_command.hpp"
#include "altair_msgs/msg/robot_control_state.hpp"
#include "altair_msgs/msg/stabilizer.hpp"
#include "quintic_walk/walk_engine.hpp"
#include "quintic_walk/walk_ik.hpp"
#include "quintic_walk/walk_stabilizer.hpp"
#include "quintic_walk/walk_visualizer.hpp"
#include "quintic_splines/abstract_ik.hpp"

#include "robotis_framework_common/motion_module.h"
#include "robotis_framework_common/singleton.h"
#include "robotis_device/robot.h"
#include "robotis_device/dynamixel.h"

#include "quintic_walk_parameters.hpp"

namespace quintic_walk
{

class QuinticWalk : public robotis_framework::MotionModule, public robotis_framework::Singleton<QuinticWalk>

{
public:
  QuinticWalk();
  virtual ~QuinticWalk();

  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, double> sensors) override;

  void stop() override;
  bool isRunning() override;

  void onModuleEnable() override;
  void onModuleDisable() override;

  altair_msgs::msg::JointCommand step(double dt);
  altair_msgs::msg::JointCommand initial_position(double dt);
  altair_msgs::msg::Stabilizer getStabilizerData();

  /**
   * Reset everything to initial idle state.
   */
  void reset();
  /**
   * Updates the parameters of the walking after a parameter change.
   */
  void updateParams();
  /**
   * Reset walk to any given state. Necessary for using this as reference in learning.
   */
  void reset(WalkState state, double phase, geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool reset_odometry);
  /**
   * Initialize internal WalkEngine to correctly zeroed, usable state
   */
  void initializeEngine();
  /**
   * Sets the current state of the robot
   * @param msg The current state
   */
  void robotStateCb(altair_msgs::msg::RobotControlState::SharedPtr msg);
  WalkEngine* getEngine();
  WalkIK* getIk();
  nav_msgs::msg::Odometry getOdometry();
  double getTimerFreq();

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& parameters);

  void publish_debug();
  void move_smoother(double& buffer, double current, double target);

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<std::thread> queue_thread_;
  std::atomic<bool> queue_thread_running_{ false };
  int control_cycle_msec_{ 0 };
  std::map<std::string, int> joint_name_to_id_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  void checkPhaseRestAndReset();
  void queueThread();
  void applyJointCommandToResult(const altair_msgs::msg::JointCommand& command,
                                 const std::map<std::string, robotis_framework::Dynamixel*>& dxls,
                                 std::size_t max_joints);
  void setJointTarget(const std::string& joint_name, double target_position,
                      const std::map<std::string, robotis_framework::Dynamixel*>& dxls);
  std::array<double, 4> get_step_from_vel(geometry_msgs::msg::Twist::SharedPtr msg);
  void stepCb(geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelCb(geometry_msgs::msg::Twist::SharedPtr msg);
  void kickCb(std_msgs::msg::Bool::SharedPtr msg);
  void jointStateCb(sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCb(sensor_msgs::msg::Imu::SharedPtr msg);
  double getTimeDelta();

  // Declare parameter listener and struct from the generate_parameter_library
  std::shared_ptr<walking::ParamListener> param_listener_;
  // quintic_walk::ParamListener param_listener_;

  // Datastructure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  walking::Params config_;
  walking::Params params_;

  std::shared_ptr<quintic_walk::WalkEngine> walk_engine_;
  std::shared_ptr<quintic_walk::WalkStabilizer> stabilizer_;
  std::shared_ptr<quintic_walk::WalkIK> ik_;
  std::shared_ptr<quintic_walk::WalkVisualizer> visualizer_;

  WalkRequest current_request_ = { .stop_walk = true };
  WalkRequest last_request_;

  bool first_run_ = true;

  double last_ros_update_time_ = 0;

  int robot_state_ = altair_msgs::msg::RobotControlState::CONTROLLABLE;
  int current_support_foot_ = biped_interfaces::msg::Phase::DOUBLE_STANCE;

  WalkResponse current_response_;
  WalkResponse current_stabilized_response_;
  quintic_splines::JointGoals motor_goals_;

  nav_msgs::msg::Odometry odom_msg_;

  double current_trunk_fused_pitch_ = 0;
  double current_trunk_fused_roll_ = 0;

  double current_fly_pressure_ = 0;
  double current_fly_effort_ = 0;

  std::optional<rclcpp::Time> last_imu_measurement_time_;
  double imu_y_acc = 0;

  bool got_new_goals_ = false;

  bool init_position_;
  std::string parameter_file_path_;
  bool parameters_loaded_from_file_{ false };

  // motor step counter used for smooth movement, max unsigned 8 bit = 256
  uint8_t mot_cont_;
  double goal_pos_temp_[12] = { 0 };

  uint8_t joint_size_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> sensor_key_;
  std::unordered_map<std::string, double> sensor_state_;
  std::unordered_map<std::string, uint8_t> joint_map_;
  std::unordered_map<std::string, double> joint_command_;
  std::unordered_map<std::string, double> joint_state_;
  std::unordered_map<std::string, double> joint_axis_direction_;

  rclcpp::Publisher<altair_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_support_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  rclcpp::Publisher<altair_msgs::msg::Stabilizer>::SharedPtr pub_stabilizer_;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_subscriber_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr step_sub_;
  rclcpp::Subscription<altair_msgs::msg::RobotControlState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kick_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace quintic_walk

namespace
{
namespace fs = std::filesystem;
bool prepareParameterArguments(std::vector<std::string>& arguments, std::string& resolved_path,
                               std::string& failure_reason)
{
  fs::path candidate_path;
  bool env_override = false;

  if (const char* env_path = std::getenv("QUINTIC_WALK_PARAM_FILE"))
  {
    if (env_path[0] != '\0')
    {
      candidate_path = fs::path(env_path);
      env_override = true;
    }
  }

  if (candidate_path.empty())
  {
    try
    {
      auto share_dir = ament_index_cpp::get_package_share_directory("quintic_walk");
      candidate_path = fs::path(share_dir) / "config" / "walking.yaml";
    }
    catch (const std::exception& e)
    {
      failure_reason = std::string("unable to locate package share directory: ") + e.what();
      return false;
    }
  }

  if (candidate_path.empty())
  {
    failure_reason = "parameter file path is empty";
    return false;
  }

  if (!fs::exists(candidate_path))
  {
    failure_reason = std::string("parameter file not found at '") + candidate_path.string() + "'";
    if (env_override)
      failure_reason += " (from QUINTIC_WALK_PARAM_FILE)";
    return false;
  }

  resolved_path = candidate_path.string();
  arguments = { "--ros-args", "--params-file", resolved_path };
  return true;
}
}  // namespace

#endif  // QUINTIC_WALK__QUINTIC_WALK_HPP_
