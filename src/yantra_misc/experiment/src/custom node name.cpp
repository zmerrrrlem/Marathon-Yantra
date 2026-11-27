#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class TestClockNode : public rclcpp::Node {
public:
  std::string node_name;
  TestClockNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : rclcpp::Node{node_name, options} {
    TestClockNode::node_name = node_name;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestClockNode>("xxx");
  RCLCPP_INFO(rclcpp::get_logger(node->node_name), "Fasdf");

  if (node == nullptr) {
    return 1;
  }

  auto executor = rclcpp::executors::SingleThreadedExecutor{};
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}