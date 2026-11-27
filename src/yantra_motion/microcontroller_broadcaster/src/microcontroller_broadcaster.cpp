#include "microcontroller_broadcaster/microcontroller_broadcaster.hpp"
#include "controller_interface/helpers.hpp"
#include <stdexcept>
#include <chrono>
#include <memory>
#include <string>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
using namespace std::chrono_literals;

namespace microcontroller_broadcaster
{
  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    params_ = param_listener_->get_params();

    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(
      semantic_components::IMUSensor(params_.sensor_name));
    try
    {
      // register ft sensor data publisher
      sensor_state_publisher_ =
        get_node()->create_publisher<sensor_msgs::msg::Imu>("~/imu", rclcpp::SystemDefaultsQoS());
      button_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Int16>("~/button", rclcpp::SystemDefaultsQoS());
      realtime_sensor_state_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
      realtime_button_state_publisher_ = std::make_unique<ButtonPublisher>(button_state_publisher_);
    }
    catch (const std::exception& e)
    {
      fprintf(
        stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
        e.what());
      return CallbackReturn::ERROR;
    }

    realtime_sensor_state_publisher_->lock();
    realtime_sensor_state_publisher_->msg_.header.frame_id = params_.frame_id;
    // convert double vector to fixed-size array in the message
    for (size_t i = 0; i < 9; ++i)
    {
      realtime_sensor_state_publisher_->msg_.orientation_covariance[i] = params_.static_covariance_orientation[i];
      realtime_sensor_state_publisher_->msg_.angular_velocity_covariance[i] =
        params_.static_covariance_angular_velocity[i];
      realtime_sensor_state_publisher_->msg_.linear_acceleration_covariance[i] =
        params_.static_covariance_linear_acceleration[i];
    }
    realtime_sensor_state_publisher_->unlock();

    realtime_button_state_publisher_->lock();
    realtime_button_state_publisher_->msg_.data = 99;
    realtime_button_state_publisher_->unlock();


    RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    imu_sensor_->release_interfaces();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn MicrocontrollerBroadcaster::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration MicrocontrollerBroadcaster::command_interface_configuration() const
  {

    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  controller_interface::InterfaceConfiguration MicrocontrollerBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = imu_sensor_->get_state_interface_names();
    state_interfaces_config.names.push_back("button/button_msg");

    return state_interfaces_config;
  }

  controller_interface::return_type MicrocontrollerBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    if (realtime_sensor_state_publisher_ && realtime_sensor_state_publisher_->trylock())
    {
      realtime_sensor_state_publisher_->msg_.header.stamp = time;
      imu_sensor_->get_values_as_message(realtime_sensor_state_publisher_->msg_);
      realtime_sensor_state_publisher_->unlockAndPublish();
    }

    if (realtime_button_state_publisher_ && realtime_button_state_publisher_->trylock())
    {
      const size_t last_index = state_interfaces_.size() - 1;
      realtime_button_state_publisher_->msg_.data = static_cast<int16_t>(state_interfaces_[last_index].get_value());
      realtime_button_state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

}  // namespace microcontroller_broadcaster
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(microcontroller_broadcaster::MicrocontrollerBroadcaster, controller_interface::ControllerInterface)