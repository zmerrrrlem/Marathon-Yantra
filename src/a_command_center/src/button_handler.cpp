#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include "yantra_msgs/msg/robot_control_state.hpp"
#include <cstdint>

class ButtonHandler : public rclcpp::Node
{
public:
  int8_t button_;
  yantra_msgs::msg::RobotControlState msg;

  ButtonHandler() : Node("button_handler")
  {
    button_subs_ = this->create_subscription<std_msgs::msg::Int8>(
      "button_state", 1,
      std::bind(&ButtonHandler::button_callback, this, std::placeholders::_1));
    robot_state_pubs_ = this->create_publisher<yantra_msgs::msg::RobotControlState>("robot_state", 1);
  }

  void button_command()
  {
    if (button_ == 1)
    {
      msg.state = yantra_msgs::msg::RobotControlState::WALKING_INIT;
      robot_state_pubs_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published WALKING_INIT state.");
    }
  }

  void button_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Button state: %d", msg->data);
    button_ = msg->data;

    button_command();
  }



private:
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr button_subs_;
  rclcpp::Publisher<yantra_msgs::msg::RobotControlState>::SharedPtr robot_state_pubs_;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ButtonHandler>();
  RCLCPP_INFO(node->get_logger(), "Button handler node started.");
  rclcpp::Rate loop_rate(1);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
