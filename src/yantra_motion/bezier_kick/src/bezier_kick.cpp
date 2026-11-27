#define M_TAU M_PI * 2
#define DEGREE2RADIAN 0,01745

#include "bezier_kick/bezier_kick.hpp"
#include "controller_interface/helpers.hpp"
#include <stdexcept>
#include <iostream>
#include <memory>
#include "geometry_msgs/msg/vector3.h"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace bezier_kick
{

  controller_interface::CallbackReturn BezierKick::on_init()
  {
    try
    {
      node_ = get_node();
      param_listener_ = std::make_shared<kicking::ParamListener>(node_);
      config_ = param_listener_->get_params();
      ik_ = std::make_shared<KickIK>();
      RCLCPP_INFO(node_->get_logger(), "on_init initialized successfully");
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BezierKick::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    std::cout << "=====================on_configure=====================" << std::endl;
    try
    {
      tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

      auto moveit_node = std::make_shared<rclcpp::Node>("kicking_moveit_node");
      std::cout << "============= Creating moveit_node ==========" << std::endl;
      // get all kinematics parameters from the move_group node if they are not set manually via constructor
      auto temp_node = std::make_shared<rclcpp::Node>("temporary_parameter_client_node");
      std::string check_kinematic_parameters;
      if (!moveit_node->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver", check_kinematic_parameters)) {
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "/move_group");
        rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({ "robot_description_kinematics" }, 10);
        auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
        for (auto& parameter : copied_parameters) {
          moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
          moveit_node->set_parameter(parameter);
          std::cout << "=====================" << parameter.get_name() << std::endl;
        }
      }

      // Load MoveIt! model
      robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node);
      kinematic_model_ = robot_model_loader_->getModel();
      if (!kinematic_model_)
      {
        RCLCPP_FATAL(get_node()->get_logger(), "No robot model loaded, killing quintic walk.");
        return controller_interface::CallbackReturn::ERROR;
      }
      else {
        RCLCPP_INFO(node_->get_logger(), "Robot model loaded.");
      }

      ik_->init(kinematic_model_);

      // Process the parameters
      updateParams();

      // Initialize Joints
      joint_names_ = { "l_ank_roll", "l_ank_pitch", "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw",
                        "r_ank_roll", "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw" };
      joint_size_ = joint_names_.size();
      // Map joint names to indices in unordered_map for easier access
      for (uint8_t i = 0; i < joint_size_; i++) { joint_map_[joint_names_[i]] = i; }

    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  void BezierKick::updateParams()
  {
    config_ = param_listener_->get_params();
  }


  controller_interface::CallbackReturn BezierKick::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BezierKick::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BezierKick::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BezierKick::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn BezierKick::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration BezierKick::command_interface_configuration() const
  {
    // claim command interface only for the specified joints
    std::cout << "=====================command_interface_configuration=====================" << std::endl;
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration BezierKick::state_interface_configuration() const
  {
    // claim state interface only for the specified joints
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::ALL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  Eigen::Vector3d BezierKick::bezierCurve2(double phase, Eigen::Vector3d start, Eigen::Vector3d end)
  {
    Eigen::Vector3d result;

    result(0) = (1 - phase) * start(0) + phase * end(0);
    result(1) = (1 - phase) * start(1) + phase * end(1);
    result(2) = (1 - phase) * start(2) + phase * end(2);

    return result;
  }

  Eigen::Vector3d BezierKick::bezierCurve3(double phase, Eigen::Vector3d start, Eigen::Vector3d inter, Eigen::Vector3d end)
  {
    Eigen::Vector3d result;

    result(0) = std::pow((1 - phase), 2) * start(0) + 2 * (1 - phase) * phase * inter(0) + std::pow(phase, 2) * end(0);
    result(1) = std::pow((1 - phase), 2) * start(1) + 2 * (1 - phase) * phase * inter(1) + std::pow(phase, 2) * end(1);
    result(2) = std::pow((1 - phase), 2) * start(2) + 2 * (1 - phase) * phase * inter(2) + std::pow(phase, 2) * end(2);

    return result;
  }

  controller_interface::return_type BezierKick::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    
  }
};  // namespace bezier_kick
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bezier_kick::BezierKick, controller_interface::ControllerInterface)
