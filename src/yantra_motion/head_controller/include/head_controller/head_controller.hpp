#ifndef HEAD_CONTROLLER__HEAD_CONTROLLER_HPP_
#define HEAD_CONTROLLER__HEAD_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <cmath>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "head_controller_parameters.hpp"

namespace head_controller
{

class HeadController : public controller_interface::ControllerInterface

{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  void move_smoother(double current, double target, double& result);

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<head_controller::ParamListener> param_listener_;
  head_controller::Params config_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, uint8_t> joint_map_;

  void updateParams();
  bool isHeadSafe(double roll, double pitch);

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr roll_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr pitch_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_mode_sub_;
  int8_t target_roll_pos_;
  int8_t target_pitch_pos_;
  std::string search_mode_;

  uint8_t speed_factor_;
  bool override_control_; 
  double roll_max_, roll_min_, pitch_max_, pitch_min_;
};

}  // namespace head_controller

#endif  // HEAD_CONTROLLER__HEAD_CONTROLLER_HPP_
