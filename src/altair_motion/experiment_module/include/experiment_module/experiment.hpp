#ifndef EXPERIMENTMODULE_HPP
#define EXPERIMENTMODULE_HPP

#include <map>
#include <thread>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_math/robotis_math.h"

namespace robotis_op
{


class ExperimentModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ExperimentModule>, public rclcpp::Node
{
 public:
  ExperimentModule();
  virtual ~ExperimentModule();

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  void stop();
  bool isRunning();

  void onModuleEnable();
  void onModuleDisable();

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::msg::String::SharedPtr msg);
  void overrideCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

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

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr override_command_sub_;
  rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_module_client_;

  std::map<std::string, int> joint_name_to_id_;
  
  bool override_active_;
  std::map<std::string, double> override_joint_positions_;
  std::mutex override_mutex_;
};

}

#endif /* EXPERIMENTMODULE_HPP */
