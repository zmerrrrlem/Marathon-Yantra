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

/* Authors: SCH, Kayman */

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <map>
#include <thread>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace robotis_op
{


class ControllerTemplate : public robotis_framework::MotionModule, public robotis_framework::Singleton<ControllerTemplate>, public rclcpp::Node
{
 public:
  ControllerTemplate();
  virtual ~ControllerTemplate();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::msg::String::SharedPtr msg);

  /* Parameter */
  BaseModuleState *base_module_state_;
  BaseJointState *joint_state_;

 private:
  void queueThread();
  void setCtrlModule(std::string module);
  void callServiceSettingModule(const std::string &module_name);
  void publishStatusMsg(unsigned int type, std::string msg);

  int control_cycle_msec_;
  std::unique_ptr<std::thread> queue_thread_;
  std::unique_ptr<std::thread> tra_gene_thread_;

  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_ctrl_module_pub_;

  rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_module_client_;

  std::map<std::string, int> joint_name_to_id_;
};

}

#endif /* BASEMODULE_H_ */
