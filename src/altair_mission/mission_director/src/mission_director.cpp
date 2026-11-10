#include "mission_director/mission_director.hpp"

#define PI 3.1415926535897932384626433832795
using namespace std::chrono_literals;

MissionDirector::MissionDirector()
{
  node_ = std::make_shared<rclcpp::Node>("mission_director_node", rclcpp::NodeOptions().use_intra_process_comms(true));
  service_node_ = std::make_shared<rclcpp::Node>("service_helper_node");
  factory_.registerSimpleAction("set_module", std::bind(&MissionDirector::set_module, this));
  factory_.registerSimpleAction("set_robot_state_controllable",
                                std::bind(&MissionDirector::set_robot_state_controllable, this));
  factory_.registerSimpleAction("set_robot_state_falling", std::bind(&MissionDirector::set_robot_state_falling, this));
  factory_.registerSimpleAction("set_robot_state_fallen", std::bind(&MissionDirector::set_robot_state_fallen, this));
  factory_.registerSimpleAction("set_robot_state_getting_up",
                                std::bind(&MissionDirector::set_robot_state_getting_up, this));
  factory_.registerSimpleAction("set_robot_state_animation_running",
                                std::bind(&MissionDirector::set_robot_state_animation_running, this));
  factory_.registerSimpleAction("set_robot_state_penalty", std::bind(&MissionDirector::set_robot_state_penalty, this));
  factory_.registerSimpleAction("set_robot_state_walking_init",
                                std::bind(&MissionDirector::set_robot_state_walking_init, this));
  factory_.registerSimpleAction("set_robot_state_walking", std::bind(&MissionDirector::set_robot_state_walking, this));
  factory_.registerSimpleAction("set_robot_state_motor_off",
                                std::bind(&MissionDirector::set_robot_state_motor_off, this));
  factory_.registerSimpleAction("set_robot_state_picked_up",
                                std::bind(&MissionDirector::set_robot_state_picked_up, this));
  factory_.registerSimpleAction("set_robot_state_kicking", std::bind(&MissionDirector::set_robot_state_kicking, this));

  std::string dir = ament_index_cpp::get_package_share_directory("mission_director");
  std::string tree_path = dir + "/config/mission_1.xml";
  tree_ = factory_.createTreeFromFile(tree_path);
  const unsigned port = 5555;
  publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, port);

  robot_state_pub_ = node_->create_publisher<altair_msgs::msg::RobotControlState>("/robot_state", 1);
  set_module_client_ =
      service_node_->create_client<robotis_controller_msgs::srv::SetModule>("/robotis/set_present_ctrl_modules");
  set_module_ = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();

  runner_ = node_->create_wall_timer(std::chrono::milliseconds(1000 / 20), std::bind(&MissionDirector::run, this));
}

MissionDirector::~MissionDirector()
{
  if (runner_)
  {
    runner_->cancel();
    runner_.reset();
  }
  node_.reset();
}

void MissionDirector::run()
{
  set_robot_state(altair_msgs::msg::RobotControlState::WALKING_INIT);
  if (!rclcpp::ok())
  {
    return;
  }
  auto status = tree_.tickOnce();

  if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(rclcpp::get_logger("MissionDirector"), "Mission completed with status: %d", static_cast<int>(status));
    switch (status)
    {
      case BT::NodeStatus::IDLE:
        RCLCPP_INFO(rclcpp::get_logger("MissionDirector"), "Node status: IDLE");
        break;
      case BT::NodeStatus::RUNNING:
        RCLCPP_INFO(rclcpp::get_logger("MissionDirector"), "Node status: RUNNING");
        break;
      case BT::NodeStatus::SUCCESS:
        RCLCPP_INFO(rclcpp::get_logger("MissionDirector"), "Node status: SUCCESS");
        break;
      case BT::NodeStatus::FAILURE:
        RCLCPP_INFO(rclcpp::get_logger("MissionDirector"), "Node status: FAILURE");
        break;
      default:
        break;
    }

    runner_->cancel();
    rclcpp::shutdown();
  }
}

template <typename client_type, typename request_type>
void service_call(rclcpp::Node::SharedPtr node, const client_type& client, const request_type& req)
{
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout << "Successfully call service\n";
    return;
  }
  else
  {
    std::cout << "Failed to call service\n";
    return;
  }
}

BT::NodeStatus MissionDirector::set_module()
{
  set_module_->module_name = "quintic_walk_module";
  service_call<rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr,
               robotis_controller_msgs::srv::SetModule::Request::SharedPtr>(service_node_, set_module_client_,
                                                                            set_module_);
  return BT::NodeStatus::SUCCESS;
}

void MissionDirector::set_robot_state(const int state)
{
  using STATE = altair_msgs::msg::RobotControlState;
  switch (state)
  {
    case STATE::CONTROLLABLE:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::CONTROLLABLE;
      break;
    case STATE::FALLING:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::FALLING;
      break;
    case STATE::FALLEN:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::FALLEN;
      break;
    case STATE::GETTING_UP:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::GETTING_UP;
      break;
    case STATE::ANIMATION_RUNNING:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::ANIMATION_RUNNING;
      break;
    case STATE::PENALTY:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::PENALTY;
      break;
    case STATE::WALKING_INIT:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::WALKING_INIT;
      break;
    case STATE::WALKING:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::WALKING;
      break;
    case STATE::MOTOR_OFF:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::MOTOR_OFF;
      break;
    case STATE::PICKED_UP:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::PICKED_UP;
      break;
    case STATE::KICKING:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::KICKING;
      break;
    default:
      robot_state_msg_.state = altair_msgs::msg::RobotControlState::CONTROLLABLE;
      break;
  }
  robot_state_pub_->publish(robot_state_msg_);
}
BT::NodeStatus MissionDirector::set_robot_state_controllable()
{
  set_robot_state(altair_msgs::msg::RobotControlState::CONTROLLABLE);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_falling()
{
  set_robot_state(altair_msgs::msg::RobotControlState::FALLING);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_fallen()
{
  set_robot_state(altair_msgs::msg::RobotControlState::FALLEN);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_getting_up()
{
  set_robot_state(altair_msgs::msg::RobotControlState::GETTING_UP);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_animation_running()
{
  set_robot_state(altair_msgs::msg::RobotControlState::ANIMATION_RUNNING);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_penalty()
{
  set_robot_state(altair_msgs::msg::RobotControlState::PENALTY);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_walking_init()
{
  set_robot_state(altair_msgs::msg::RobotControlState::WALKING_INIT);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_walking()
{
  set_robot_state(altair_msgs::msg::RobotControlState::WALKING);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_motor_off()
{
  set_robot_state(altair_msgs::msg::RobotControlState::MOTOR_OFF);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_picked_up()
{
  set_robot_state(altair_msgs::msg::RobotControlState::PICKED_UP);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MissionDirector::set_robot_state_kicking()
{
  set_robot_state(altair_msgs::msg::RobotControlState::KICKING);
  return BT::NodeStatus::SUCCESS;
}
