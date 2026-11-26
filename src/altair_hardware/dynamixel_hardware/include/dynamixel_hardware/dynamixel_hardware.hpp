// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ALTAIR_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define ALTAIR_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <map>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "rclcpp/macros.hpp"

#include "dynamixel_msgs/msg/dynamixel_state.hpp"
#include "dynamixel_msgs/srv/get_data_from_dxl.hpp"
#include "dynamixel_msgs/srv/set_data_to_dxl.hpp"
#include "dynamixel_msgs/srv/reboot_dxl.hpp"
#include "dynamixel_msgs/srv/set_torque.hpp"
#include "dynamixel_msgs/srv/set_ping.hpp"

#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_buffer.h"

#include "std_srvs/srv/set_bool.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
#define TORQUE_ON  1 
#define TORQUE_OFF 0

namespace altair_hardware {
  struct JointValue
  {
    double position{ 0.0 };
    double velocity{ 0.0 };
    double effort{ 0.0 };
  };

  struct Joint
  {
    JointValue state{};
    JointValue command{};
    JointValue prev_command{};
  };

  enum class ControlMode
  {
    Position,
    Velocity,
    Torque,
    Currrent,
    ExtendedPosition,
    MultiTurn,
    CurrentBasedPosition,
    PWM,
  };

  class DynamixelHardware : public hardware_interface::SystemInterface, rclcpp::Node
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

    DynamixelHardware();
    ~DynamixelHardware();

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    rclcpp::Logger logger_;
    return_type enable_torque(const bool enabled);
    return_type enable_torque(const bool enabled, const uint8_t* id_arr, size_t count);

    return_type set_control_mode(const ControlMode& mode, const bool force_set = false);
    return_type set_position_mode();
    std::vector<double> minimum_jerk_trajectory(double init_pos, double target_pos, double total_time, double dt);
    return_type set_init_position();
    return_type init_position_command();

    return_type reset_command();
    void move_smoother(double& current, double target);
    return_type go_to_init();
    CallbackReturn write_to_servo();
    CallbackReturn set_joint_velocities();
    CallbackReturn set_joint_params();
    CallbackReturn set_joint_offset();
    return_type stop();

    DynamixelWorkbench dynamixel_workbench_;
    std::map<const char* const, const ControlItem*> control_items_;
    
    static constexpr size_t MAX_JOINTS = 20;  // Define maximum number of joints

    // Replace vectors with stack arrays
    std::array<Joint, MAX_JOINTS> joints_;
    std::array<uint8_t, MAX_JOINTS> joint_ids_;
    std::array<double, MAX_JOINTS> joint_offset_;
    std::array<double, MAX_JOINTS> target_positions_;
    size_t joint_count_{ 0 };  // Track actual number of joints
    
    bool torque_enabled_{ false };
    ControlMode control_mode_{ ControlMode::Position };
    bool mode_changed_{ false };
    bool sim_mode_{ false };
    bool tuning_mode_{ false };
    bool init_position_{ false };
    bool sync_read_error_flag_{ false };
    uint8_t mot_cont_{ 100 };

    rclcpp::Service<dynamixel_msgs::srv::GetDataFromDxl>::SharedPtr get_dxl_data_srv_;
    void get_dxl_data_srv_callback(
      const std::shared_ptr<dynamixel_msgs::srv::GetDataFromDxl::Request> request,
      std::shared_ptr<dynamixel_msgs::srv::GetDataFromDxl::Response> response);

    rclcpp::Service<dynamixel_msgs::srv::SetDataToDxl>::SharedPtr set_dxl_data_srv_;
    void set_dxl_data_srv_callback(
      const std::shared_ptr<dynamixel_msgs::srv::SetDataToDxl::Request> request,
      std::shared_ptr<dynamixel_msgs::srv::SetDataToDxl::Response> response);

    rclcpp::Service<dynamixel_msgs::srv::RebootDxl>::SharedPtr reboot_dxl_srv_;
    void reboot_dxl_srv_callback(
      const std::shared_ptr<dynamixel_msgs::srv::RebootDxl::Request> request,
      std::shared_ptr<dynamixel_msgs::srv::RebootDxl::Response> response);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_dxl_torque_srv_;
    void set_dxl_torque_srv_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    rclcpp::Service<dynamixel_msgs::srv::SetTorque>::SharedPtr set_dxl_torque_id_srv_;
    void set_dxl_torque_id_srv_callback(
      const std::shared_ptr<dynamixel_msgs::srv::SetTorque::Request> request,
      std::shared_ptr<dynamixel_msgs::srv::SetTorque::Response> response);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_init_pose_srv_;
    void set_init_pose_srv_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    rclcpp::Service<dynamixel_msgs::srv::SetPing>::SharedPtr set_ping_srv_;
    void set_ping_srv_callback(
      const std::shared_ptr<dynamixel_msgs::srv::SetPing::Request> request,
      std::shared_ptr<dynamixel_msgs::srv::SetPing::Response> response);
  };
}  // namespace altair_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
