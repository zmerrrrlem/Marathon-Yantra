#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "experiment_parameters.hpp"

using namespace std::chrono_literals;



int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("experiment", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  experiment::ParamListener param_listener_ = experiment::ParamListener(node);
  experiment::Params config_ = param_listener_.get_params();
  std::cout << config_.engine.freq << std::endl;

  return 0;
}