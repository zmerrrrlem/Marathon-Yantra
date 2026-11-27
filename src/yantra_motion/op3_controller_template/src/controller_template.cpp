/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman, SCH */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <robotis_controller_msgs/msg/status_msg.hpp>
#include <robotis_controller_msgs/srv/set_module.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include "op3_controller_template/controller_template.h"

namespace robotis_op
{

BaseModule::BaseModule()
  : Node("op3_controller_template"),
    control_cycle_msec_(0),
    has_goal_joints_(false),
    ini_pose_only_(false),
    init_pose_file_path_("")
{
  enable_ = false;
  module_name_ = "controller_template";
  control_mode_ = robotis_framework::PositionControl;

  base_module_state_ = new BaseModuleState();
  joint_state_ = new BaseJointState();
}

BaseModule::~BaseModule()
{
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();

  for (auto& [joint_name, state] : result_)
    delete state;

  delete base_module_state_;
  delete joint_state_;
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_msec_ = control_cycle_msec;

  // init result, joint_id_table
  for (const auto& [joint_name, dxl_info] : robot->dxls_)
  {
    joint_name_to_id_[joint_name] = dxl_info->id_;
    result_[joint_name] = new robotis_framework::DynamixelState();
    result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
  }

  /* Load ROS Parameter */
  this->declare_parameter<std::string>("init_pose_file_path", ament_index_cpp::get_package_share_directory("op3_controller_template") + "/data/ini_pose.yaml");
  init_pose_file_path_ = this->get_parameter("init_pose_file_path").as_string();

  /* publish topics */
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("/robotis/status", 1);
  set_ctrl_module_pub_ = this->create_publisher<std_msgs::msg::String>("/robotis/enable_ctrl_module", 1);

  queue_thread_ = std::make_unique<std::thread>(&BaseModule::queueThread, this);
}

void BaseModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* subscribe topics */
  auto ini_pose_msg_sub = this->create_subscription<std_msgs::msg::String>("/robotis/base/ini_pose", 5, 
                                    std::bind(&BaseModule::initPoseMsgCallback, this, std::placeholders::_1));
  set_module_client_ = this->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");

  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void BaseModule::initPoseMsgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (base_module_state_->is_moving_ == false)
  {
    if (msg->data == "ini_pose")
    {
      // set module of all joints -> this module
      callServiceSettingModule(module_name_);

      // wait for changing the module to controller_template and getting the goal position
      rclcpp::Rate wait_rate(125);
      while (enable_ == false || has_goal_joints_ == false)
        wait_rate.sleep();

      // parse initial pose
      parseInitPoseData(init_pose_file_path_);

      // generate trajectory
      if (tra_gene_thread_ && tra_gene_thread_->joinable())
        tra_gene_thread_->join();
      tra_gene_thread_ = std::make_unique<std::thread>(&BaseModule::initPoseTrajGenerateProc, this);
    }
  }
  else
    RCLCPP_INFO(this->get_logger(), "previous task is alive");

  return;
}

bool BaseModule::isRunning()
{
  return base_module_state_->is_moving_;
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (const auto& [joint_name, dxl] : dxls)
  {
    if (result_.find(joint_name) == result_.end())
      continue;

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
    joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
  }

}

void BaseModule::stop()
{
  return;
}

void BaseModule::onModuleEnable()
{
  RCLCPP_INFO(this->get_logger(), "Base Module is enabled");
}

void BaseModule::onModuleDisable()
{
  has_goal_joints_ = false;
}

void BaseModule::setCtrlModule(std::string module)
{
  std_msgs::msg::String control_msg;
  control_msg.data = module_name_;

  set_ctrl_module_pub_->publish(control_msg);
}

void BaseModule::callServiceSettingModule(const std::string &module_name)
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

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
  auto status_msg = robotis_controller_msgs::msg::StatusMsg();
  status_msg.header.stamp = rclcpp::Clock().now();
  status_msg.type = type;
  status_msg.module_name = "Base";
  status_msg.status_msg = msg;

  status_msg_pub_->publish(status_msg);
}
}
