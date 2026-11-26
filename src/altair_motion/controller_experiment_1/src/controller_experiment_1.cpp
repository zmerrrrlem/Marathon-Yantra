#include "controller_experiment_1/controller_experiment_1.hpp"
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

namespace controller_experiment_1
{
  controller_interface::CallbackReturn ControllerExperiment1::on_init()
  {
    try
    {
      node_ = get_node();
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

  controller_interface::CallbackReturn ControllerExperiment1::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    std::cout << "configure controller_experiment_1" << std::endl;
    chatter_subsriber_ = node_->create_subscription<std_msgs::msg::String>(
      "chatter",
      10,
      std::bind(&ControllerExperiment1::chatterCb, this, std::placeholders::_1));

    chatter_publisher_ = node_->create_publisher<std_msgs::msg::String>("chatter", 10);

    // // Create dummy node for moveit
    // auto moveit_node = std::make_shared<rclcpp::Node>("walking_moveit_node", "/quintic_walk", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    // std::cout << "=====================after moveit node=====================" << std::endl;

    // // get all kinematics parameters from the move_group node if they are not set manually via constructor
    // auto temp_node = std::make_shared<rclcpp::Node>("temporary_parameter_client_node", "/temporary");
    // std::string check_kinematic_parameters;
    // if (!moveit_node->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver",
    //   check_kinematic_parameters)) {

    //   auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "/move_group");
    //   rcl_interfaces::msg::ListParametersResult parameter_list =
    //     parameters_client->list_parameters({ "robot_description_kinematics" }, 10);
    //   auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    //   for (auto& parameter : copied_parameters) {
    //     moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
    //     moveit_node->set_parameter(parameter);
    //     std::cout << "=====================" << parameter.get_name() << std::endl;
    //   }
    // }

    // // // Load MoveIt! model
    // auto robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node, "robot_description");

    // moveit::core::RobotModelPtr kinematic_model_ = robot_model_loader_->getModel();


    // if (!kinematic_model_) {
    //   RCLCPP_FATAL(node_->get_logger(), "No robot model loaded, killing quintic walk.");
    // }
    // else {
    //   RCLCPP_INFO(node_->get_logger(), "Robot model loaded.");
    // }

    joint_names_ = { "l_ank_roll", "l_ank_pitch", "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "r_ank_roll", "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw" };
    for (uint8_t i = 0; i < joint_names_.size(); i++) {
      joint_map_[joint_names_[i]] = i;
    }

    return CallbackReturn::SUCCESS;
  }

  void ControllerExperiment1::chatterCb(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  controller_interface::CallbackReturn ControllerExperiment1::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    // ...
    std::cout << "activate controller_experiment_1" << std::endl;
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment1::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment1::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment1::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ControllerExperiment1::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration ControllerExperiment1::command_interface_configuration() const
  {

    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ControllerExperiment1::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    return state_interfaces_config;
  }

  controller_interface::return_type ControllerExperiment1::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    // for (size_t i = 0; i < joint_names_.size(); ++i)
    // {
    //   command_interfaces_[joint_map_[]].set_value(joint_position_commands_[i]);
    // }
    command_interfaces_[joint_map_["r_knee"]].set_value(45 * 0.01745);


    std::cout << "update controller_experiment_1" << std::endl;
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, ROS 2! Message";
    // chatter_publisher_->publish(message);

    return controller_interface::return_type::OK;
  }

}  // namespace controller_experiment_1
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(controller_experiment_1::ControllerExperiment1, controller_interface::ControllerInterface)