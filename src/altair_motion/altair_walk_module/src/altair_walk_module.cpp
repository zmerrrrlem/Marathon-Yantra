#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <robotis_controller_msgs/msg/status_msg.hpp>
#include <robotis_controller_msgs/srv/set_module.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include "altair_walk_module/altair_walk_module.hpp"

// Example: 
// set motion module all joint: ros2 service call /robotis/set_present_ctrl_modules robotis_controller_msgs/srv/SetModule "{module_name: 'altair_walk_module'}"
// set motion module per-joint: ros2 service call /robotis/set_present_joint_ctrl_modules robotis_controller_msgs/srv/SetJointModule "{joint_name: ['head_pan', 'head_tilt'], module_name: ['altair_walk_module', 'altair_walk_module']}"
// Send goal position to servo: ros2 topic pub -r 10 /robotis/override_command sensor_msgs/msg/JointState "{name: ['head_pan', 'head_tilt'], position: [0.0, -0.3]}"

namespace altair_walk_module
{

AltairWalkModule::AltairWalkModule()
  : Node("altair_walk_module"),
    control_cycle_msec_(0),
    override_active_(false)
{
  enable_ = false;
  module_name_ = "altair_walk_module";
  control_mode_ = robotis_framework::PositionControl;
}

AltairWalkModule::~AltairWalkModule()
{
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();

  for (auto& [joint_name, state] : result_)
    delete state;

}

void AltairWalkModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  RCLCPP_INFO(this->get_logger(), "AltairWalkModule: initialize");
  control_cycle_msec_ = control_cycle_msec;

  // init result, joint_id_table
  for (const auto& [joint_name, dxl_info] : robot->dxls_)
  {
    joint_name_to_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 1);
  
  queue_thread_ = std::make_unique<std::thread>(&AltairWalkModule::queueThread, this);
}

void AltairWalkModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* subscribe topics */
  set_module_client_ = this->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");
  
  override_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/altair_walk/joint_state", 1,
    std::bind(&AltairWalkModule::joint_state_cb, this, std::placeholders::_1));

  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void AltairWalkModule::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(override_mutex_);
  
  override_joint_positions_.clear();
  
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (i < msg->position.size())
    {
      const std::string& joint_name = msg->name[i];
      
      // Only accept joints that exist in our joint_name_to_id_ mapping
      if (joint_name_to_id_.find(joint_name) != joint_name_to_id_.end())
      {
        override_joint_positions_[joint_name] = msg->position[i];
        RCLCPP_DEBUG(this->get_logger(), "Override joint %s (id: %d) to position %f", 
                     joint_name.c_str(), joint_name_to_id_[joint_name], msg->position[i]);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unknown joint name in override command: %s", joint_name.c_str());
      }
    }
  }
  
  override_active_ = !override_joint_positions_.empty();
  
  if (override_active_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Override command received for %zu joints", override_joint_positions_.size());
  }
}

bool AltairWalkModule::isRunning()
{
  return false;
}

void AltairWalkModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- apply override positions if active -----*/
  {
    std::lock_guard<std::mutex> lock(override_mutex_);
    if (override_active_)
    {
      for (const auto& [joint_name, position] : override_joint_positions_)
      {
        if (result_.find(joint_name) != result_.end())
        {
          result_[joint_name]->goal_position_ = position;
        }
      }
    }
  }
}

void AltairWalkModule::stop()
{
  return;
}

void AltairWalkModule::onModuleEnable()
{
  RCLCPP_INFO(this->get_logger(), "altair_walk_module is enabled");
}

void AltairWalkModule::onModuleDisable()
{
  return;
}

void AltairWalkModule::setCtrlModule(std::string module)
{
  std_msgs::msg::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_->publish(control_msg);
}

void AltairWalkModule::callServiceSettingModule(const std::string &module_name)
{
  auto request = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
  request->module_name = module_name;

  if (!set_module_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Service not available");
    return;
  }

  auto future = set_module_client_->async_send_request(request,
      [this](rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedFuture result)
      {
        RCLCPP_INFO(this->get_logger(), "callServiceSettingModule : result : %d", result.get()->result);
      });
}

void AltairWalkModule::publishStatusMsg(unsigned int type, std::string msg)
{
  auto status_msg = robotis_controller_msgs::msg::StatusMsg();
  status_msg.header.stamp = rclcpp::Clock().now();
  status_msg.type = type;
  status_msg.module_name = "Experiment";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}
}
