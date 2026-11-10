#include "mission_director/mission_director.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  auto mission_director = std::make_shared<MissionDirector>();
  executor->add_node(mission_director->node_);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
