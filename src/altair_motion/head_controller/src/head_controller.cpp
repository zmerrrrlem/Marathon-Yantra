#include "head_controller/head_controller.hpp"
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
using namespace std::chrono_literals;

#define DEGREE2RADIAN 0.01745
constexpr double LIMIT_HARDWARE_ROLL_MAX = 80 * DEGREE2RADIAN;
constexpr double LIMIT_HARDWARE_ROLL_MIN = -80 * DEGREE2RADIAN;
constexpr double LIMIT_HARDWARE_PITCH_MAX = 60 * DEGREE2RADIAN;
constexpr double LIMIT_HARDWARE_PITCH_MIN = -60 * DEGREE2RADIAN;

namespace head_controller
{
  controller_interface::CallbackReturn HeadController::on_init()
  {
    try
    {
      node_ = get_node();
      param_listener_ = std::make_shared<head_controller::ParamListener>(get_node());
      config_ = param_listener_->get_params();
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn HeadController::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    std::cout << "configure head_controller" << std::endl;

    // set manual control with parameter "ros2 param set /head_controller override_control true"
    override_control_ = false;

    roll_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "head_controller/roll", 10,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        target_roll_pos_ = msg->data;
      });

    pitch_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
      "head_controller/pitch", 10,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        target_pitch_pos_ = msg->data;
      });

    search_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "head_controller/search_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        search_mode_ = msg->data;
      });


    // Smaller value is faster, higher is smoother
    speed_factor_ = config_.speed_factor;

    joint_names_ = { "head_pan", "head_tilt" };
    for (uint8_t i = 0; i < joint_names_.size(); i++) {
      joint_map_[joint_names_[i]] = i;
    }

    return CallbackReturn::SUCCESS;
  }

  void HeadController::updateParams()
  {
    config_ = param_listener_->get_params();
    override_control_ = config_.override_control;
    speed_factor_ = config_.speed_factor;
    roll_max_ = config_.limit.roll_max * DEGREE2RADIAN;
    roll_min_ = config_.limit.roll_min * DEGREE2RADIAN;
    pitch_max_ = config_.limit.pitch_max * DEGREE2RADIAN;
    pitch_min_ = config_.limit.pitch_min * DEGREE2RADIAN;
  }

  bool HeadController::isHeadSafe(double roll, double pitch)
  {
    // Simple bounding box check
    
    if (roll < LIMIT_HARDWARE_ROLL_MIN || roll > LIMIT_HARDWARE_ROLL_MAX)
      return false;
    if (pitch < LIMIT_HARDWARE_PITCH_MIN || pitch > LIMIT_HARDWARE_PITCH_MAX)
      return false;

    return true;
  }


  controller_interface::CallbackReturn HeadController::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    // ...
    std::cout << "activate head_controller" << std::endl;
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn HeadController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn HeadController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn HeadController::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn HeadController::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration HeadController::command_interface_configuration() const
  {

    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration HeadController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::return_type HeadController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    updateParams();

    double current_roll_pos = state_interfaces_[joint_map_["head_pan"]].get_value();
    double current_pitch_pos = state_interfaces_[joint_map_["head_tilt"]].get_value();

    if (override_control_) {
      double roll_rad = target_roll_pos_ * DEGREE2RADIAN;
      double pitch_rad = target_pitch_pos_ * DEGREE2RADIAN;
      if (!isHeadSafe(roll_rad, pitch_rad)) {
        RCLCPP_WARN(node_->get_logger(), "Head pose (roll=%.2f, pitch=%.2f) is out of safe range! Command ignored.", roll_rad, pitch_rad);
        return controller_interface::return_type::OK;
      }

      command_interfaces_[joint_map_["head_pan"]].set_value(roll_rad);
      command_interfaces_[joint_map_["head_tilt"]].set_value(pitch_rad);
      return controller_interface::return_type::OK;
    }




    // --- Search Modes ---
    static double target_roll_pos = 0.0;
    static double target_pitch_pos = 0.0;

    // Swing mode variables
    static double swing_target_roll = roll_max_;
    static int swing_direction = 1;
    static bool swing_initialized = false;

    // Square mode variables
    static int square_corner = 0;
    static const double corners[4][2] = {
      {roll_min_, pitch_min_},
      {roll_max_, pitch_min_},
      {roll_max_, pitch_max_},
      {roll_min_, pitch_max_}
    };
    static bool square_initialized = false;

    // Sweep mode variables
    static double sweep_target_roll = roll_max_;
    static double sweep_pitch = pitch_min_;
    static int sweep_direction = 1;
    static int sweep_pitch_direction = 1;
    static bool sweep_initialized = false;

    // Nod mode variables
    static double nod_target_pitch = pitch_max_;
    static int nod_direction = 1;
    static bool nod_initialized = false;

    if (search_mode_ == "swing") {
      if (!swing_initialized) {
        swing_target_roll = roll_max_;
        swing_direction = 1;
        swing_initialized = true;
        square_initialized = sweep_initialized = nod_initialized = false;
      }
      target_roll_pos = swing_target_roll;
      target_pitch_pos = -20 * DEGREE2RADIAN;
      if (fabs(current_roll_pos - swing_target_roll) < 0.01) {
        swing_direction *= -1;
        swing_target_roll = (swing_direction > 0) ? roll_max_ : roll_min_;
      }
    }
    else if (search_mode_ == "square") {
      if (!square_initialized) {
        square_corner = 0;
        square_initialized = true;
        swing_initialized = sweep_initialized = nod_initialized = false;
      }
      target_roll_pos = corners[square_corner][0];
      target_pitch_pos = corners[square_corner][1];
      if (fabs(current_roll_pos - target_roll_pos) < 0.05 &&
        fabs(current_pitch_pos - target_pitch_pos) < 0.05) {
        square_corner = (square_corner + 1) % 4;
      }
    }
    else if (search_mode_ == "sweep") {
      if (!sweep_initialized) {
        sweep_target_roll = roll_max_;
        sweep_pitch = pitch_min_;
        sweep_direction = 1;
        sweep_pitch_direction = 1;
        sweep_initialized = true;
        swing_initialized = square_initialized = nod_initialized = false;
      }
      target_roll_pos = sweep_target_roll;
      target_pitch_pos = sweep_pitch;
      if (fabs(current_roll_pos - sweep_target_roll) < 0.01) {
        sweep_direction *= -1;
        sweep_target_roll = (sweep_direction > 0) ? roll_max_ : roll_min_;
        sweep_pitch += 20 * DEGREE2RADIAN * sweep_pitch_direction;
        if (sweep_pitch > pitch_max_) {
          sweep_pitch = pitch_max_;
          sweep_pitch_direction = -1;
        }
        if (sweep_pitch < pitch_min_) {
          sweep_pitch = pitch_min_;
          sweep_pitch_direction = 1;
        }
      }
    }
    else if (search_mode_ == "nod") {
      if (!nod_initialized) {
        nod_target_pitch = pitch_max_;
        nod_direction = -1;
        nod_initialized = true;
        swing_initialized = square_initialized = sweep_initialized = false;
      }
      target_roll_pos = 0.0;
      target_pitch_pos = nod_target_pitch;

      if (fabs(current_pitch_pos - nod_target_pitch) < 0.01) {
        nod_direction *= -1;
        nod_target_pitch = (nod_direction > 0) ? pitch_max_ : pitch_min_;
      }
    }
    else if (search_mode_ == "center") {
      target_roll_pos = 0;
      target_pitch_pos = 0;
      swing_initialized = square_initialized = sweep_initialized = nod_initialized = false;
    }
    else {
      target_roll_pos = current_roll_pos;
      target_pitch_pos = current_pitch_pos;
      swing_initialized = square_initialized = sweep_initialized = nod_initialized = false;
    }

    double result_roll_pos, result_pitch_pos;
    move_smoother(current_roll_pos, target_roll_pos, result_roll_pos);
    move_smoother(current_pitch_pos, target_pitch_pos, result_pitch_pos);

    // RCLCPP_INFO(node_->get_logger(),
    //   "CMD: result_roll=%.2f result_pitch=%.2f | target_roll=%.2f target_pitch=%.2f | state_roll=%.2f state_pitch=%.2f | mode=%s",
    //   result_roll_pos, result_pitch_pos, target_roll_pos, target_pitch_pos, current_roll_pos, current_pitch_pos, search_mode_.c_str());

    // Safety check
    if (!isHeadSafe(result_roll_pos, target_pitch_pos)) {
      RCLCPP_INFO(node_->get_logger(), "Head pose (roll=%.2f, pitch=%.2f) is out of safe range! Command ignored.", result_roll_pos, result_pitch_pos);
      return controller_interface::return_type::OK;
    }

    command_interfaces_[joint_map_["head_pan"]].set_value(result_roll_pos);
    command_interfaces_[joint_map_["head_tilt"]].set_value(result_pitch_pos);



    return controller_interface::return_type::OK;
  }

  void HeadController::move_smoother(double current, double target, double& result) {
    // Arguments: current: reference to current servo position, target: target servo position, result: reference to store the result
    // Optional steps parameter to control smoothing rate (higher = smoother but slower)
    double delta = target - current;
    if (fabs(delta) < 0.01) {
      result = target;
    }
    else {
      result = current + delta / speed_factor_;
    }
  }

}  // namespace head_controller
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(head_controller::HeadController, controller_interface::ControllerInterface)