#define M_TAU M_PI * 2

#include "cubic_walk/cubic_walk.hpp"
#include "controller_interface/helpers.hpp"
#include <stdexcept>
#include <cmath> // For M_PI
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/vector3.h"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace cubic_walk
{

  controller_interface::CallbackReturn CubicWalk::on_init()
  {
    try
    {
      node_ = get_node();
      param_listener_ = std::make_shared<walking::ParamListener>(node_);
      config_ = param_listener_->get_params();
      walk_engine_ = std::make_shared<WalkEngine>(node_, config_.engine);
      stabilizer_ = std::make_shared<WalkStabilizer>(node_);
      ik_ = std::make_shared<WalkIK>(node_, config_.node.ik);
      visualizer_ = std::make_shared<WalkVisualizer>(node_, config_.node.tf);
      RCLCPP_INFO(node_->get_logger(), "on_init initialized successfully");
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CubicWalk::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    std::cout << "=====================on_configure=====================" << std::endl;
    try
    {
      auto moveit_node = std::make_shared<rclcpp::Node>("walking_moveit_node");
      // Provide robot_description to the MoveIt model loader from the controller node
      // Fetch robot_description from robot_state_publisher (Gazebo does not inject it into controllers)
      {
        auto temp_node2 = std::make_shared<rclcpp::Node>("temporary_rsp_client_node");
        auto rsp_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node2, "/robot_state_publisher");
        if (rsp_client->wait_for_service(std::chrono::seconds(5))) {
          auto params = rsp_client->get_parameters({"robot_description"});
          if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            std::string rd = params[0].as_string();
            RCLCPP_INFO(node_->get_logger(), "robot_description len from rsp: %zu", rd.size());
            moveit_node->declare_parameter("robot_description", rd);
            // Load SRDF for MoveIt model
            try {
              std::string srdf_path = ament_index_cpp::get_package_share_directory("altair_moveit_config") + "/config/altair.srdf";
              std::ifstream f(srdf_path);
              std::stringstream ss; ss << f.rdbuf();
              moveit_node->declare_parameter("robot_description_semantic", ss.str());
            } catch (...) {
              RCLCPP_WARN(node_->get_logger(), "Could not load SRDF");
            }
            // Load kinematics params
            try {
              std::string kin_path = ament_index_cpp::get_package_share_directory("altair_moveit_config") + "/config/kinematics.yaml";
              YAML::Node kin = YAML::LoadFile(kin_path);
              for (auto it = kin.begin(); it != kin.end(); ++it) {
                std::string group = it->first.as<std::string>();
                for (auto pit = it->second.begin(); pit != it->second.end(); ++pit) {
                  std::string key = pit->first.as<std::string>();
                  if (pit->second.IsScalar()) {
                    std::string val = pit->second.as<std::string>();
                    try {
                      size_t pos; double d = std::stod(val, &pos);
                      if (pos == val.size()) {
                        try { moveit_node->declare_parameter("robot_description_kinematics." + group + "." + key, d); }
                        catch(...) { moveit_node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + "." + key, d)); }
                      } else {
                        try { moveit_node->declare_parameter("robot_description_kinematics." + group + "." + key, val); }
                        catch(...) { moveit_node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + "." + key, val)); }
                      }
                    } catch(...) {
                      try { moveit_node->declare_parameter("robot_description_kinematics." + group + "." + key, val); }
                      catch(...) { moveit_node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + group + "." + key, val)); }
                    }
                  }
                }
              }
            } catch (...) {
              RCLCPP_WARN(node_->get_logger(), "Could not load kinematics.yaml");
            }
          } else {
            RCLCPP_WARN(node_->get_logger(), "robot_state_publisher returned no robot_description");
          }
        } else {
          RCLCPP_WARN(node_->get_logger(), "robot_state_publisher not available; skipping robot_description fetch");
        }
      }
      // get all kinematics parameters from the move_group node if they are not set manually via constructor
      auto temp_node = std::make_shared<rclcpp::Node>("temporary_parameter_client_node");
      std::string check_kinematic_parameters;
      if (!moveit_node->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver", check_kinematic_parameters)) {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "/move_group");
        if (parameters_client->wait_for_service(std::chrono::seconds(2))) {
          rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({ "robot_description_kinematics" }, 10);
          auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
          for (auto& parameter : copied_parameters) {
            moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
            moveit_node->set_parameter(parameter);
          }
        } else {
          RCLCPP_WARN(node_->get_logger(), "move_group not available; skipping kinematics param sync.");
        }
      }

      // Load MoveIt! model
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node, "robot_description");
      kinematic_model_ = robot_model_loader_->getModel();
      if (!kinematic_model_)
      {
        RCLCPP_FATAL(get_node()->get_logger(), "No robot model loaded, killing cubic walk.");
        return controller_interface::CallbackReturn::ERROR;
      }
      else {
        RCLCPP_INFO(node_->get_logger(), "Robot model loaded.");
      }

      // Process the parameters
      updateParams();

      /* init publisher and subscriber */
      pub_controller_command_ = node_->create_publisher<altair_msgs::msg::JointCommand>("walking_motor_goals", 1);
      // pub_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
      pub_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("walk_engine_odometry", 1);
      pub_support_ = node_->create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 1);
      pub_walk_status_ = node_->create_publisher<std_msgs::msg::Int32>("altair_walk/walk_status", 1);
      step_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("step", 1, std::bind(&CubicWalk::stepCb, this, _1));
      cmd_vel_sub_ =
        node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&CubicWalk::cmdVelCb, this, _1));
      robot_state_sub_ = node_->create_subscription<altair_msgs::msg::RobotControlState>(
        "robot_state", 1, std::bind(&CubicWalk::robotStateCb, this, _1));
      imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu_sensor_broadcaster/imu", 1, std::bind(&CubicWalk::imuCb, this, _1));
      joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&CubicWalk::jointStateCb, this, _1));
      kick_sub_ = node_->create_subscription<std_msgs::msg::Bool>("kick", 1, std::bind(&CubicWalk::kickCb, this, _1));

      turn_request_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/altair_turn/request", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (turn_state_ == TURN_TURNING || turn_state_ == TURN_SETTLING) return;
          if (!has_imu_data_) {
            RCLCPP_ERROR(node_->get_logger(), "Turn request ignored: no IMU data received yet");
            return;
          }
          turn_target_yaw_ = turn_current_yaw_ + msg->data;
          turn_state_ = TURN_TURNING;
          turn_settle_count_ = 0;
          turn_start_time_ = get_node()->get_clock()->now();
          prev_turn_err_ = 0.0;
          last_turn_cmd_time_ = get_node()->get_clock()->now();
        });
      turn_state_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/altair_turn/state", 10);
      target_yaw_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/target_yaw", 10);
      node_->declare_parameter<double>("turn.kp", 1.0);
      node_->declare_parameter<double>("turn.kd", 0.0);
      node_->declare_parameter<double>("turn.max_angular", 1.0);

      ik_->init(kinematic_model_);
      visualizer_->init(kinematic_model_);

      current_state_.reset(new moveit::core::RobotState(kinematic_model_));
      current_state_->setToDefaultValues();

      walk_engine_->reset();


      // Publish the starting support state once, especially for odometry.
      // We always start with the same foot
      biped_interfaces::msg::Phase sup_state;
      sup_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
      sup_state.header.stamp = get_node()->get_clock()->now();
      pub_support_->publish(sup_state);

      this->initializeEngine();

      // Initialize Joints
      joint_names_ = { "left_leg_roll_5_joint", "left_leg_pitch_5_joint", "left_leg_pitch_3_joint", "left_leg_pitch_1_joint", "left_leg_roll_1_joint", "left_leg_yaw_1_joint",
                        "right_leg_roll_6_joint", "right_leg_pitch_6_joint", "right_leg_pitch_4_joint", "right_leg_pitch_2_joint", "right_leg_roll_2_joint", "right_leg_yaw_2_joint",
                        "left_arm_pitch_3_joint", "left_arm_pitch_1_joint", "left_arm_roll_1_joint", "left_claw_joint",
                        "right_arm_pitch_4_joint", "right_arm_pitch_2_joint", "right_arm_roll_2_joint", "right_claw_joint" };
      // "l_sho_pitch", "l_sho_roll", "l_el", "r_sho_pitch", "r_sho_roll", "r_el" 
      joint_size_ = joint_names_.size();
      // Map joint names to indices in unordered_map for easier access
      for (uint8_t i = 0; i < joint_size_; i++) { joint_map_[joint_names_[i]] = i; }

      init_position_ = true;
      // Used for initial position smoothing, higher more smooth but slower
      mot_cont_ = 100;
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void CubicWalk::updateParams()
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

  void CubicWalk::publish_debug() { visualizer_->publishDebug(current_stabilized_response_, current_state_, motor_goals_); }

  altair_msgs::msg::JointCommand CubicWalk::step(double dt)
  {

    WalkRequest request(current_request_);

    // update walk engine response
    if (got_new_goals_) {
      got_new_goals_ = false;
      walk_engine_->setGoals(request);
    }
    checkPhaseRestAndReset();
    current_response_ = walk_engine_->update(dt);


    // only calculate joint goals from this if the engine is not idle
    current_response_.current_fused_roll = current_trunk_fused_roll_;
    current_response_.current_fused_pitch = current_trunk_fused_pitch_;

    altair_msgs::msg::JointCommand command;
    if (walk_engine_->getState() != WalkState::IDLE) {
      // get stabilized goals from stabilizer
      current_stabilized_response_ =
        stabilizer_->stabilize(current_response_, rclcpp::Duration::from_nanoseconds(1e9 * dt));

      // compute motor goals from IK
      motor_goals_ = ik_->calculate(current_stabilized_response_);

      // Arm swing (optional, controlled by config)
      if (config_.engine.arm_swing_enabled) {
        auto arm_goals = ik_->getArmSwingGoals(
          current_response_.phase,
          config_.engine.arm_shoulder_pitch_amp,
          config_.engine.arm_shoulder_roll_amp,
          config_.engine.arm_elbow_bend
        );
        for (const auto& [name, pos] : arm_goals) {
          motor_goals_.first.push_back(name);
          motor_goals_.second.push_back(pos);
        }
      }

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

  altair_msgs::msg::JointCommand CubicWalk::step(double dt, const geometry_msgs::msg::Twist::SharedPtr cmdvel_msg,
    const sensor_msgs::msg::Imu::SharedPtr imu_msg,
    const sensor_msgs::msg::JointState::SharedPtr jointstate_msg) {
    // method for python interface. take all messages as parameters instead of using ROS
    cmdVelCb(cmdvel_msg);
    imuCb(imu_msg);
    jointStateCb(jointstate_msg);
    // we don't use external robot state
    current_request_.walkable_state = true;
    // update walk engine response
    altair_msgs::msg::JointCommand joint_goals = step(dt);
    return joint_goals;
  }

  double CubicWalk::getTimeDelta()
  {
    // compute time delta depended if we are currently in simulation or reality
    double dt;
    double current_ros_time = get_node()->get_clock()->now().seconds();
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

  void CubicWalk::reset()
  {
    walk_engine_->reset();
    stabilizer_->reset();
  }

  void CubicWalk::reset(WalkState state, double phase, geometry_msgs::msg::Twist::SharedPtr cmd_vel,
    bool reset_odometry)
  {
    auto step = get_step_from_vel(cmd_vel);
    bool stop_walk = cmd_vel->angular.x < 0;
    walk_engine_->reset(state, phase, step, stop_walk, true, reset_odometry);
    stabilizer_->reset();
    cmdVelCb(cmd_vel);
  }

  geometry_msgs::msg::PoseArray CubicWalk::step_open_loop(double dt,
    const geometry_msgs::msg::Twist::SharedPtr cmdvel_msg)
  {
    cmdVelCb(cmdvel_msg);
    // get cartesian goals from open loop engine
    WalkRequest request(current_request_);
    walk_engine_->setGoals(request);
    current_response_ = walk_engine_->update(dt);

    // change goals from support foot based coordinate system to trunk based coordinate system
    tf2::Transform trunk_to_support_foot_goal = current_response_.support_foot_to_trunk.inverse();
    tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * current_response_.support_foot_to_flying_foot;
    geometry_msgs::msg::Pose left_foot_goal_msg;
    geometry_msgs::msg::Pose right_foot_goal_msg;
    // decide which foot is which
    if (current_response_.is_left_support_foot)
    {
      cubic_walk::tf_pose_to_msg(trunk_to_support_foot_goal, left_foot_goal_msg);
      cubic_walk::tf_pose_to_msg(trunk_to_flying_foot_goal, right_foot_goal_msg);
    }
    else
    {
      cubic_walk::tf_pose_to_msg(trunk_to_support_foot_goal, right_foot_goal_msg);
      cubic_walk::tf_pose_to_msg(trunk_to_flying_foot_goal, left_foot_goal_msg);
    }
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.poses = { left_foot_goal_msg, right_foot_goal_msg };
    return pose_array;
  }

  geometry_msgs::msg::Pose CubicWalk::get_left_foot_pose()
  {
    moveit::core::RobotStatePtr goal_state = ik_->get_goal_state();
    geometry_msgs::msg::Pose pose;
    tf2::convert(goal_state->getGlobalLinkTransform("left_leg_roll_5"), pose);
    return pose;
  }

  geometry_msgs::msg::Pose CubicWalk::get_right_foot_pose()
  {
    moveit::core::RobotStatePtr goal_state = ik_->get_goal_state();
    geometry_msgs::msg::Pose pose;
    tf2::convert(goal_state->getGlobalLinkTransform("right_leg_roll_6"), pose);
    return pose;
  }

  std::array<double, 4> CubicWalk::get_step_from_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
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
      RCLCPP_WARN(get_node()->get_logger(),
        "Commanded step 'x: %.2f y: %.2f z: %.2f angular: %.2f' exceeded limits, clamped to 'x: %.2f y: %.2f "
        "z: "
        "%.2f angular: %.2f'",
        step[0], step[1], step[2], step[3], clamped_step[0], clamped_step[1], clamped_step[2], clamped_step[3]);
    }

    return clamped_step;
  }

  void CubicWalk::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!current_state_ || !kinematic_model_) return;
    if (msg->position.size() != msg->name.size()) {
      RCLCPP_WARN(node_->get_logger(), "JointState position/name size mismatch, ignoring message");
      return;
    }
    std::vector<std::string> names = msg->name;
    std::vector<double> goals = msg->position;
    for (size_t i = 0; i < names.size(); i++) {
      // besides its name, this method only changes a single joint position...
      const moveit::core::JointModel* jm = current_state_->getJointModel(names[i]);
      if (jm) {
        current_state_->setJointPositions(names[i], &goals[i]);
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5,
          "Joint '%s' not found in URDF, skipping joint state update", names[i].c_str());
      }
    }

    // compute the effort that is currently on the flying leg to check if it has ground contact
    // only if we have the necessary data
    if (msg->effort.size() == msg->name.size()) {
      double effort_sum = 0;
      const std::vector<std::string>& fly_joint_names =
        (walk_engine_->isLeftSupport()) ? ik_->getRightLegJointNames() : ik_->getLeftLegJointNames();
      for (size_t i = 0; i < names.size(); i++) {
        // add effort on this joint to sum, if it is part of the flying leg
        if (std::find(fly_joint_names.begin(), fly_joint_names.end(), names[i]) != fly_joint_names.end()) {
          effort_sum = effort_sum + abs(msg->effort[i]);
        }
      }
      current_fly_effort_ = effort_sum;
    }
    else {
      RCLCPP_WARN_ONCE(node_->get_logger(),
        "Joint states have no effort information. Phase reset based on this will not work.");
    }
  }

  void CubicWalk::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double roll, pitch, yaw;

    // IF orientation.w == -2, the IMU sent orientation in euler angle. We do this because our BNO IMU sent orientation in euler angle
    if (msg->orientation.w == -2) {
      roll = msg->orientation.x;
      pitch = msg->orientation.y;
      yaw = msg->orientation.z;
      rot_conv::FusedAngles imu_fused = rot_conv::FusedFromEuler(yaw, pitch, roll);

      current_trunk_fused_pitch_ = imu_fused.fusedPitch;
      current_trunk_fused_roll_ = imu_fused.fusedRoll;
    }
    // ELSE, the IMU sent orientation in quaternion
    else {
      // the incoming geometry_msgs::msg::Quaternion is transformed to a tf2::Quaternion
      tf2::Quaternion quat;
      tf2::convert(msg->orientation, quat);
      // the tf2::Quaternion has a method to access roll pitch and yaw

      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

      Eigen::Quaterniond imu_orientation_eigen = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
      rot_conv::FusedAngles imu_fused = rot_conv::FusedFromQuat(imu_orientation_eigen);

      current_trunk_fused_pitch_ = imu_fused.fusedPitch;
      current_trunk_fused_roll_ = imu_fused.fusedRoll;
    }

    turn_current_yaw_ = yaw;
    has_imu_data_ = true;

    // get angular velocities
    double roll_vel = msg->angular_velocity.x;
    double pitch_vel = msg->angular_velocity.y;

    // Calculate ema (exponential moving average) of y (sideways) acceleration in a time-independet manner
    if (last_imu_measurement_time_) {
      auto time_delta = rclcpp::Time(msg->header.stamp) - last_imu_measurement_time_.value();
      double exponent_in_s = -time_delta.nanoseconds() / (config_.node.imu_y_acc_tau * 1e9);
      double alpha = 1 - std::exp(exponent_in_s);

      imu_y_acc = msg->linear_acceleration.y * alpha + imu_y_acc * (1 - alpha);
    }
    else {
      imu_y_acc = msg->linear_acceleration.y;
    }

    if (config_.node.stability_stop.imu.active) {
      // compute the pitch offset to the currently wanted pitch of the engine
      double wanted_pitch = walk_engine_->getWantedTrunkPitch();

      // Get the sub struct of the imu stability stop config
      auto params = config_.node.stability_stop.imu;

      // Check if we have to stop the walk
      double pitch_delta = pitch - wanted_pitch;
      if (abs(roll) > params.roll.threshold or abs(pitch_delta) > params.pitch.threshold or
        abs(pitch_vel) > params.pitch.vel_threshold or abs(roll_vel) > params.roll.vel_threshold) {
        walk_engine_->requestPause();
        if (abs(roll) > params.roll.threshold) {
          RCLCPP_WARN(node_->get_logger(), "imu roll angle stop");
        }
        else if (abs(pitch_delta) > params.pitch.threshold) {
          RCLCPP_WARN(node_->get_logger(), "imu pitch angle stop");
        }
        else if (abs(pitch_vel) > params.pitch.vel_threshold) {
          RCLCPP_WARN(node_->get_logger(), "imu roll vel stop");
        }
        else {
          RCLCPP_WARN(node_->get_logger(), "imu pitch vel stop");
        }
      }
    }
    // Store the timestamp
    last_imu_measurement_time_ = msg->header.stamp;
  }

  void CubicWalk::stepCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    current_request_.linear_orders = { msg->linear.x, msg->linear.y, msg->linear.z };
    current_request_.angular_z = msg->angular.z;
    current_request_.single_step = true;
    current_request_.stop_walk = true;
    got_new_goals_ = true;
  }

  void CubicWalk::cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    got_new_goals_ = true;
    current_request_.single_step = false;

    // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
    // other axis.

    // the engine expects orders in [m] not [m/s]
    auto step = get_step_from_vel(msg);
    current_request_.linear_orders = { step[0], step[1], step[2] };
    // Turn handling: override angular_z with correction toward target when turning
    if (turn_state_ == TURN_TURNING || turn_state_ == TURN_SETTLING) {
      double err = std::atan2(std::sin(turn_current_yaw_ - turn_target_yaw_), std::cos(turn_current_yaw_ - turn_target_yaw_));
      double kp = node_->get_parameter("turn.kp").as_double();
      double kd = node_->get_parameter("turn.kd").as_double();
      double max_ang = node_->get_parameter("turn.max_angular").as_double();
      auto now = get_node()->get_clock()->now();
      double dt = (now - last_turn_cmd_time_).seconds();
      last_turn_cmd_time_ = now;
      if (dt <= 0.0) {
        dt = 0.005;
      }
      double correction = -kp * err - kd * (err - prev_turn_err_) / dt;
      prev_turn_err_ = err;
      correction = std::max(-max_ang, std::min(max_ang, correction));
      current_request_.angular_z = correction;
    } else {
      prev_turn_err_ = 0.0;
      current_request_.angular_z = step[3];
    }

    // stop_walk should be true when all velocity components are effectively zero
    // Threshold is 1e-3 m/s / rad/s to ignore noise-level commands from deprecated
    // altair_walk utilities (heading_hold, etc.) that publish small non-zero values.
    current_request_.stop_walk = (std::fabs(msg->linear.x) < 1e-3 &&
                                 std::fabs(msg->linear.y) < 1e-3 &&
                                 std::fabs(msg->linear.z) < 1e-3 &&
                                 std::fabs(msg->angular.z) < 1e-3);
  }

  void CubicWalk::checkPhaseRestAndReset()
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

  void CubicWalk::robotStateCb(const altair_msgs::msg::RobotControlState::SharedPtr msg)
  {
    robot_state_ = msg->state;
  }

  void CubicWalk::kickCb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    walk_engine_->requestKick(msg->data);
  }

  nav_msgs::msg::Odometry CubicWalk::getOdometry()
  {
    // odometry to trunk is transform to support foot * transform from support to trunk
    tf2::Transform support_foot_tf;
    if (walk_engine_->isLeftSupport()) {
      support_foot_tf = walk_engine_->getLeft();
    }
    else {
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

    return odom_msg_;
  }

  void CubicWalk::initializeEngine()
  {
    walk_engine_->reset();
  }

  WalkEngine* CubicWalk::getEngine()
  {
    return walk_engine_.get();
  }

  WalkIK* CubicWalk::getIk()
  {
    return ik_.get();
  }

  moveit::core::RobotModelPtr* CubicWalk::get_kinematic_model()
  {
    return &kinematic_model_;
  }

  double CubicWalk::getTimerFreq()
  {
    return config_.node.engine_freq;
  }


  void CubicWalk::move_smoother(double& current, double target) {
    // Arguments: current: reference to current servo position, target: target servo position
    // Optional steps parameter to control smoothing rate (higher = smoother but slower)
    if (mot_cont_ < 1) {
      current = target;
    }
    else {
      current += (target - current) / mot_cont_;
    }
  }

  controller_interface::CallbackReturn CubicWalk::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    walk_engine_->reset();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CubicWalk::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CubicWalk::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CubicWalk::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn CubicWalk::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration CubicWalk::command_interface_configuration() const
  {
    // claim command interface only for the specified joints
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration CubicWalk::state_interface_configuration() const
  {
    // claim state interface only for the specified joints
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::ALL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }

    return config;
  }

  controller_interface::return_type CubicWalk::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Get up to date parameters
    updateParams();

    // std::cout << "=====================update=====================" << std::endl;

    double dt = getTimeDelta();
    // necessary as timer in simulation does not work correctly https://github.com/ros2/rclcpp/issues/465
    if (dt != 0.0) {

      if (robot_state_ == altair_msgs::msg::RobotControlState::FALLING ||
        robot_state_ == altair_msgs::msg::RobotControlState::GETTING_UP) {
        // the robot fell, we have to reset everything and do nothing else
        walk_engine_->reset();
        stabilizer_->reset();
      }
      else {
        // we don't want to walk, even if we have orders, if we are not in the right state
        /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
         * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
         // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
        current_request_.walkable_state = robot_state_ == altair_msgs::msg::RobotControlState::CONTROLLABLE ||
          robot_state_ == altair_msgs::msg::RobotControlState::WALKING ||
          robot_state_ == altair_msgs::msg::RobotControlState::MOTOR_OFF;

        // reset when we start walking, otherwise PID controller will use old I value
        if ((last_request_.linear_orders[0] == 0 && last_request_.linear_orders[1] == 0 &&
          last_request_.angular_z == 0) &&
          (current_request_.linear_orders[0] != 0 || current_request_.linear_orders[1] != 0 ||
            current_request_.angular_z != 0)) {
          stabilizer_->reset();
        }
        last_request_ = current_request_;

        // Turn state machine (minimal)
        if (turn_state_ == TURN_TURNING || turn_state_ == TURN_SETTLING) {
          double err = std::atan2(std::sin(turn_current_yaw_ - turn_target_yaw_), std::cos(turn_current_yaw_ - turn_target_yaw_));
          double elapsed = (get_node()->get_clock()->now() - turn_start_time_).seconds();

          // Abort on stale IMU data (no update for more than 1.0 second)
          bool imu_stale = false;
          if (last_imu_measurement_time_) {
            double imu_age = (get_node()->get_clock()->now() - last_imu_measurement_time_.value()).seconds();
            if (imu_age > 1.0) {
              imu_stale = true;
            }
          } else {
            imu_stale = true;
          }

          // Abort if the walk engine is not actively walking
          bool engine_stopped = (walk_engine_->getState() == WalkState::IDLE ||
                                 walk_engine_->getState() == WalkState::PAUSED);

          if (imu_stale || engine_stopped) {
            turn_state_ = TURN_ABORTED;
            turn_settle_count_ = 0;
            if (imu_stale) {
              RCLCPP_WARN(node_->get_logger(), "Turn aborted: stale IMU data");
            } else {
              RCLCPP_WARN(node_->get_logger(), "Turn aborted: walk engine stopped");
            }
          } else if (elapsed > 60.0) {
            turn_state_ = TURN_ABORTED;
            turn_settle_count_ = 0;
          } else {
            double capture = 0.139626; // 8 deg
            double tol = 0.0349066;   // 2 deg
            if (turn_state_ == TURN_TURNING && std::fabs(err) < capture) {
              turn_state_ = TURN_SETTLING;
              turn_settle_count_ = 0;
            } else if (turn_state_ == TURN_SETTLING) {
              if (std::fabs(err) <= tol) turn_settle_count_++;
              else turn_settle_count_ = 0;
              if (turn_settle_count_ >= 10) {
                turn_state_ = TURN_DONE;
                turn_settle_count_ = 0;
              }
            }
          }
        } else if (turn_state_ == TURN_DONE) {
          double tol = 0.0349066; // 2 deg
          double err = std::atan2(std::sin(turn_current_yaw_ - turn_target_yaw_), std::cos(turn_current_yaw_ - turn_target_yaw_));
          if (std::fabs(err) <= tol) {
            turn_settle_count_++;
          } else {
            turn_settle_count_ = 0;
          }
          if (turn_settle_count_ >= 10) {
            turn_state_ = TURN_IDLE;
            turn_settle_count_ = 0;
          }
        } else if (turn_state_ == TURN_ABORTED) {
          turn_settle_count_++;
          if (turn_settle_count_ >= 10) {
            turn_state_ = TURN_IDLE;
            turn_settle_count_ = 0;
          }
        }

        if (turn_state_pub_) {
          std_msgs::msg::Int32 st; st.data = turn_state_;
          turn_state_pub_->publish(st);
        }
        if (target_yaw_pub_ && (turn_state_ == TURN_TURNING || turn_state_ == TURN_SETTLING)) {
          std_msgs::msg::Float64 t; t.data = turn_target_yaw_;
          target_yaw_pub_->publish(t);
        }

        // perform all the actual calculations
        altair_msgs::msg::JointCommand joint_goals = step(dt);

        // only publish goals if we are not idle
        if (walk_engine_->getState() != WalkState::IDLE) {
          pub_controller_command_->publish(joint_goals);
          init_position_ = false;

          // Passing joints command value to the shared memory hardware_interface, minus 6 to exclude the arm
          for (uint8_t i = 0; i < joint_goals.joint_names.size(); ++i) {
            // access the joint name from the joint_goals msg step(dt) and set the value to the corresponding joint index
            auto joint_it = joint_map_.find(joint_goals.joint_names[i]);
            if (joint_it == joint_map_.end()) {
              RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5,
                "Unknown joint name in IK output: %s, skipping command",
                joint_goals.joint_names[i].c_str());
              continue;
            }
            (void)command_interfaces_[joint_it->second].set_value(joint_goals.positions[i]);
          }
          // publish current support state
          biped_interfaces::msg::Phase support_state;
          if (walk_engine_->isDoubleSupport()) {
            support_state.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
          }
          else if (walk_engine_->isLeftSupport()) {
            support_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
          }
          else {
            support_state.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
          }
          // publish if foot changed
          if (current_support_foot_ != support_state.phase) {
            support_state.header.stamp = node_->get_clock()->now();
            pub_support_->publish(support_state);
            current_support_foot_ = support_state.phase;
          }
        }
      }
      // publish debug information
      if (config_.node.debug_active) {
        publish_debug();
      }
      // always publish odometry to not confuse odometry fuser
      pub_odometry_->publish(getOdometry());
      // Publish walk status for compatibility with altair_walk consumers
      std_msgs::msg::Int32 ws;
      ws.data = static_cast<int>(walk_engine_->getState());
      pub_walk_status_->publish(ws);
    }

    return controller_interface::return_type::OK;
  }

}  // namespace cubic_walk
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_walk::CubicWalk, controller_interface::ControllerInterface)
