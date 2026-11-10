#include "quintic_walk/quintic_walk.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace quintic_walk
{

QuinticWalk::QuinticWalk()
{

  module_name_ = "quintic_walk_module";
  control_mode_ = robotis_framework::PositionControl;
  enable_ = false;

  std::vector<std::string> parameter_arguments;
  std::string parameter_failure_reason;
  rclcpp::NodeOptions node_options;

  if (prepareParameterArguments(parameter_arguments, parameter_file_path_, parameter_failure_reason))
  {
    node_options.arguments(parameter_arguments);
    parameters_loaded_from_file_ = true;
  }

  // Use the node name that matches the parameter file key ("quintic_walk") so
  // parameters from config/walking.yaml are loaded correctly via --params-file.
  node_ = std::make_shared<rclcpp::Node>("quintic_walk", node_options);

  if (parameters_loaded_from_file_)
  {
    RCLCPP_INFO(node_->get_logger(), "Loaded quintic walk parameters from %s", parameter_file_path_.c_str());
  }
  else if (!parameter_failure_reason.empty())
  {
    RCLCPP_WARN(node_->get_logger(),
                "Falling back to default quintic walk parameters because the YAML file could not be loaded: %s",
                parameter_failure_reason.c_str());
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(),
                "Falling back to default quintic walk parameters because no parameter file was provided.");
  }


  init_position_ = true;
  mot_cont_ = 100;
  joint_size_ = 0;
}

QuinticWalk::~QuinticWalk()
{
  queue_thread_running_.store(false);
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();
  queue_thread_.reset();

  for (auto& entry : result_)
    delete entry.second;
}

void QuinticWalk::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  if (queue_thread_ && queue_thread_->joinable())
  {
    queue_thread_running_.store(false);
    queue_thread_->join();
    queue_thread_.reset();
  }

  control_cycle_msec_ = control_cycle_msec;

  // Initialize result map
  for (const auto& [joint_name, dxl_info] : robot->dxls_)
  {
    joint_name_to_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  try
  {
    param_listener_ = std::make_shared<walking::ParamListener>(node_);
    config_ = param_listener_->get_params();
    walk_engine_ = std::make_shared<WalkEngine>(node_, config_.engine);
    stabilizer_ = std::make_shared<WalkStabilizer>(node_);
    ik_ = std::make_shared<WalkIK>(node_, config_.node.ik);
    visualizer_ = std::make_shared<WalkVisualizer>(node_, config_.node.tf);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(node_->get_logger(), "Failed to initialize QuinticWalk components: %s", e.what());
    throw;
  }

  updateParams();

  pub_controller_command_ = node_->create_publisher<altair_msgs::msg::JointCommand>("walking_motor_goals", 1);
  pub_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("walk_engine_odometry", 1);
  pub_support_ = node_->create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 1);
  pub_stabilizer_ = node_->create_publisher<altair_msgs::msg::Stabilizer>("stabilizer_data", 1);

  step_sub_ =
      node_->create_subscription<geometry_msgs::msg::Twist>("step", 1, std::bind(&QuinticWalk::stepCb, this, _1));
  cmd_vel_sub_ =
      node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&QuinticWalk::cmdVelCb, this, _1));
  robot_state_sub_ = node_->create_subscription<altair_msgs::msg::RobotControlState>(
      "robot_state", 1, std::bind(&QuinticWalk::robotStateCb, this, _1));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("robotis_op3/imu", rclcpp::SensorDataQoS(),
                                                               std::bind(&QuinticWalk::imuCb, this, _1));
  std::cout << "Imu topic at robotis_op3/imu\n";
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(), std::bind(&QuinticWalk::jointStateCb, this, _1));
  kick_sub_ = node_->create_subscription<std_msgs::msg::Bool>("kick", 1, std::bind(&QuinticWalk::kickCb, this, _1));

  ik_->init();

  walk_engine_->reset();

  biped_interfaces::msg::Phase sup_state;
  sup_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
  sup_state.header.stamp = node_->get_clock()->now();
  pub_support_->publish(sup_state);

  this->initializeEngine();

  joint_names_ = { "l_ank_roll",  "l_ank_pitch", "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw",
                   "r_ank_roll",  "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw",
                   "l_sho_pitch", "l_sho_roll",  "l_el",   "r_sho_pitch", "r_sho_roll", "r_el" };
  joint_size_ = joint_names_.size();
  for (uint8_t i = 0; i < joint_size_; i++)
    joint_map_[joint_names_[i]] = i;

  init_position_ = true;
  mot_cont_ = 100;

  // pinocchio::urdf::buildModel(urdf_path, model_);

  queue_thread_running_.store(true);
  queue_thread_ = std::make_unique<std::thread>(&QuinticWalk::queueThread, this);

  RCLCPP_INFO(node_->get_logger(), "QuinticWalk: INITIALIZE");
}

void QuinticWalk::queueThread()
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_->get_node_base_interface());
  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok() && queue_thread_running_.load())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void QuinticWalk::stop()
{
  walk_engine_->reset();
  stabilizer_->reset();
}

bool QuinticWalk::isRunning()
{
  return walk_engine_->getState() != WalkState::IDLE;
}

void QuinticWalk::onModuleEnable()
{
  std::cout << "onModuleEnable" << std::endl;
  RCLCPP_INFO(node_->get_logger(), "Quintic walk module enabled");
}

void QuinticWalk::onModuleDisable()
{
  stop();
}

void QuinticWalk::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                          std::map<std::string, double> /*sensors*/)
{
  if (enable_ == false)
  {
    // RCLCPP_INFO(node_->get_logger(), "enable==false");
    return;
  }

  updateParams();
  double dt = getTimeDelta();
  if (dt == 0.0)
  {
    // RCLCPP_INFO(node_->get_logger(), "dt==0");
    return;
  }

  if (robot_state_ == altair_msgs::msg::RobotControlState::FALLING ||
      robot_state_ == altair_msgs::msg::RobotControlState::GETTING_UP)
  {
    // RCLCPP_INFO(node_->get_logger(), "reset");
    walk_engine_->reset();
    stabilizer_->reset();
  }
  else
  {
    // RCLCPP_INFO(node_->get_logger(), "current_request_.walkable_state");
    current_request_.walkable_state = robot_state_ == altair_msgs::msg::RobotControlState::CONTROLLABLE ||
                                      robot_state_ == altair_msgs::msg::RobotControlState::WALKING ||
                                      robot_state_ == altair_msgs::msg::RobotControlState::MOTOR_OFF;

    if ((last_request_.linear_orders[0] == 0 && last_request_.linear_orders[1] == 0 && last_request_.angular_z == 0) &&
        (current_request_.linear_orders[0] != 0 || current_request_.linear_orders[1] != 0 ||
         current_request_.angular_z != 0))
    {
      stabilizer_->reset();
    }
    last_request_ = current_request_;

    altair_msgs::msg::JointCommand joint_goals = step(dt);

    if (robot_state_ == altair_msgs::msg::RobotControlState::WALKING_INIT)
    {
      // RCLCPP_INFO(node_->get_logger(), "WALKING_INIT");;
      altair_msgs::msg::JointCommand init_goals = initial_position(dt);

      if (mot_cont_ == 100)
      {
        for (uint8_t i = 0; i < joint_size_ - 6 && i < init_goals.joint_names.size(); ++i)
        {
          auto dxl_it = dxls.find(init_goals.joint_names[i]);
          if (dxl_it == dxls.end())
            continue;
          goal_pos_temp_[i] = dxl_it->second->dxl_state_->present_position_;
        }
      }

      for (uint8_t i = 0; i < joint_size_ - 6 && i < init_goals.joint_names.size(); ++i)
      {
        auto dxl_it = dxls.find(init_goals.joint_names[i]);
        if (dxl_it == dxls.end())
          continue;
        double current_position = dxl_it->second->dxl_state_->present_position_;
        move_smoother(goal_pos_temp_[i], current_position, init_goals.positions[i]);
        setJointTarget(init_goals.joint_names[i], goal_pos_temp_[i], dxls);
      }

      pub_controller_command_->publish(init_goals);
      if (mot_cont_ > 0)
        --mot_cont_;
      else
      {
        init_position_ = false;
        robot_state_ = altair_msgs::msg::RobotControlState::CONTROLLABLE;
        mot_cont_ = 100;
      }
    }

    if (walk_engine_->getState() != WalkState::IDLE)
    {
      // RCLCPP_INFO(node_->get_logger(), "!= WalkState::IDLE");
      pub_controller_command_->publish(joint_goals);
      init_position_ = false;

      std::size_t leg_joint_count = joint_size_ > 6 ? joint_size_ - 6 : joint_size_;
      applyJointCommandToResult(joint_goals, dxls, leg_joint_count);

      double r_sho_pitch = 0.0;
      double l_sho_pitch = 0.0;
      walk_engine_->computeArmAngle(r_sho_pitch, l_sho_pitch);
      setJointTarget("r_sho_pitch", r_sho_pitch, dxls);
      setJointTarget("l_sho_pitch", l_sho_pitch, dxls);

      biped_interfaces::msg::Phase support_state;
      if (walk_engine_->isDoubleSupport())
        support_state.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
      else if (walk_engine_->isLeftSupport())
        support_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
      else
        support_state.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;

      if (current_support_foot_ != support_state.phase)
      {
        support_state.header.stamp = node_->get_clock()->now();
        pub_support_->publish(support_state);
        current_support_foot_ = support_state.phase;
      }
    }
  }

  if (config_.node.debug_active)
    publish_debug();
  pub_odometry_->publish(getOdometry());

  if(config_.node.debug_active)
    pub_stabilizer_->publish(getStabilizerData());

}

} // namespace quintic_walk
