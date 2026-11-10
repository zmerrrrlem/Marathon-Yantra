#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "op3_demo/soccer_demo.hpp"

using namespace std::chrono_literals;

// Global variables
bool manager_is_ready = false;
std::shared_ptr<altair_mission::SoccerDemo> soccer_demo_node;

void statusCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg)
{
  if (msg->status_msg == "Finish Init Pose")
  {
    manager_is_ready = true;
    RCLCPP_INFO(rclcpp::get_logger("demo_node"), "Manager is ready!");
  }
}

bool checkManagerRunning(rclcpp::Node::SharedPtr node, const std::string& manager_name)
{
  auto node_names = node->get_node_names();
  
  for (const auto& name : node_names)
  {
    if (name == manager_name)
    {
      RCLCPP_INFO(node->get_logger(), "Manager found!");
      if (manager_is_ready)
      {
        RCLCPP_INFO(node->get_logger(), "Manager ready!");
        return true;
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "Manager not ready!");
        return false;
      }
    }
  }
  
  RCLCPP_ERROR(node->get_logger(), "Can't find manager node");
  return false;
}

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create a temporary node for initialization
  auto init_node = rclcpp::Node::make_shared("demo_init_node");
  
  RCLCPP_INFO(init_node->get_logger(), "Starting OP3 Soccer Demo Node");
  
  // Subscribe to manager status
  auto status_sub = init_node->create_subscription<robotis_controller_msgs::msg::StatusMsg>(
      "/robotis/status", 10, statusCallback);
  
  // Wait for manager to be running
  std::string manager_name = "/altair_manager";
  rclcpp::Rate rate(1s);
  
  RCLCPP_INFO(init_node->get_logger(), "Waiting for manager node...");
  
  while (rclcpp::ok())
  {
    rclcpp::spin_some(init_node);
    
    if (checkManagerRunning(init_node, manager_name))
    {
      RCLCPP_INFO(init_node->get_logger(), "Successfully connected to manager");
      break;
    }
    
    RCLCPP_WARN(init_node->get_logger(), "Waiting for manager...");
    rate.sleep();
  }
  
  // Give manager a moment to fully initialize
  rclcpp::sleep_for(1s);
  
  // Create soccer demo node
  RCLCPP_INFO(init_node->get_logger(), "Creating soccer demo node...");
  soccer_demo_node = std::make_shared<altair_mission::SoccerDemo>();
  
  // Enable soccer mode
  rclcpp::sleep_for(500ms);
  soccer_demo_node->setModuleEnable();
  
  RCLCPP_INFO(soccer_demo_node->get_logger(), "Soccer demo node is running!");
  RCLCPP_INFO(soccer_demo_node->get_logger(), "Publish to /ball_tracker/position to provide ball position");
  RCLCPP_INFO(soccer_demo_node->get_logger(), "Publish to /soccer_demo/command with 'start' or 'stop'");
  
  // Create multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(soccer_demo_node);
  executor.add_node(init_node);
  
  // Spin
  executor.spin();
  
  // Cleanup
  rclcpp::shutdown();
  return 0;
}
