#include <functional>
#include "inverse_kinematic/inverse_kinematic.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace inverse_kinematic {

  InverseKinematic::InverseKinematic(rclcpp::Node::SharedPtr node) : node_(node)
  {
  }

  void InverseKinematic::setup() {
  }


}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  // init node
  auto node = std::make_shared<rclcpp::Node>("inverser_kinematic", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  // Create an executor that will be used to execute callbacks for our node.
  rclcpp::experimental::executors::EventsExecutor exec;

  // Init node part of the walking
  inverse_kinematic::InverseKinematic IK_node(node);
  IK_node.setup();

  // Create timer that calls the run method of the walking regularly
  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 100);
  rclcpp::TimerBase::SharedPtr timer =
    rclcpp::create_timer(node, node->get_clock(), timer_duration, [&IK_node]() -> void { IK_node.run(); });

  // Run the executor so everything is executed
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
