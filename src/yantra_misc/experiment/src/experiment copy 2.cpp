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

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = [this]() -> void {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(this->count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("quintic_walk_node");
  std::cout << "=====================before moveit node=====================" << std::endl;
  auto moveit_node = std::make_shared<rclcpp::Node>("walking_moveit_node");
  std::cout << "=====================after moveit node=====================" << std::endl;

  // get all kinematics parameters from the move_group node if they are not set manually via constructor
  auto temp_node = std::make_shared<rclcpp::Node>("temporary_parameter_client_node");
  std::string check_kinematic_parameters;
  if (!moveit_node->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver",
    check_kinematic_parameters)) {

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temp_node, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10 * 1e9,
        "Can't copy parameters from move_group node. Service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({ "robot_description_kinematics" }, 10);
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    for (auto& parameter : copied_parameters) {
      moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
      moveit_node->set_parameter(parameter);
      std::cout << "=====================" << parameter.get_name() << std::endl;
    }
  }


  // std::this_thread::sleep_for(std::chrono::seconds(5));
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node);
  std::cout << "=====================robot_model_loader=====================" << std::endl;
  moveit::core::RobotModelPtr kinematic_model_ = robot_model_loader_->getModel();
  RCLCPP_INFO(node_->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
  std::cout << "=====================load model=====================" << std::endl;


  if (!kinematic_model_)
  {
    std::cout << "No robot model loaded, killing quintic walk." << std::endl;
  }

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model_));
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("LeftLeg");

  // Specify the group name
  std::string group_name = "LeftLeg";

  // Retrieve the Joint Model Group
  if (!joint_model_group)
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint model group '%s' not found.", group_name.c_str());
  }

  // Check for the Kinematics Solver
  auto solver_instance = joint_model_group->getSolverInstance();
  std::cout << "=====================solver_instance=====================" << typeid(*solver_instance).name() << std::endl;

#include <thread>
#include <chrono>
  // std::this_thread::sleep_for(std::chrono::seconds(40));
  // rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}