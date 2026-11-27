#include "controller_experiment_2/controller_experiment_2.hpp"
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

namespace controller_experiment_2
{
  controller_interface::CallbackReturn ControllerExperiment2::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<zmp_walk::ParamListener>(get_node());
      params_ = param_listener_->get_params();
      RCLCPP_ERROR(get_node()->get_logger(), "initttttttttt");
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment2::on_configure(const rclcpp_lifecycle::State& previous_state)
  {

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment2::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment2::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment2::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment2::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment2::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration ControllerExperiment2::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration ControllerExperiment2::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    return state_interfaces_config;
  }

  controller_interface::return_type ControllerExperiment2::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    // for (const auto& id : dxls_key_)
    // {
    //   command_interfaces_[dxls_index_[id]].set_value(dxls_command_[id]);
    //   std::cout << id << " : " << dxls_command_[id] << std::endl;
    // }
    std::cout << "update controller_experiment_2" << std::endl;

    return controller_interface::return_type::OK;
  }

}  // namespace controller_experiment_2
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controller_experiment_2::ControllerExperiment2, controller_interface::ControllerInterface)