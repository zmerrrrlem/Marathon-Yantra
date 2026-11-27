#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include "sensor_msgs/msg/joint_state.hpp"


const char* port_name = "/dev/ttyUSB0";
int baud_rate = 1000000;

uint16_t model_number = 0;
uint8_t joint_ids_[20] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
uint8_t left_hand_id[3] = { 2, 4, 6 };
uint8_t right_hand_id[3] = { 1, 3, 5 };
uint8_t left_leg_id[6] = { 8, 10, 12, 14, 16, 18 };
uint8_t right_leg_id[6] = { 7, 9, 11, 13, 15, 17 };
uint8_t head_id[2] = { 19, 20 };

// Add limb size constants
const size_t left_hand_size = 3;
const size_t right_hand_size = 3;
const size_t left_leg_size = 6;
const size_t right_leg_size = 6;
const size_t head_size = 2;

int dxl_cnt = 20;
constexpr const char* kDynamixelHardware = "DynamixelHardware";

constexpr const char* kGoalPositionItem = "Goal_Position";
constexpr const char* kPresentPositionItem = "Present_Position";
constexpr const char* kPresentVelocityItem = "Present_Velocity";
int32_t joint_velocities[20] = { 0 };
int32_t joint_positions[20] = { 0 };

DynamixelWorkbench dynamixel_workbench_;
std::map<const char* const, const ControlItem*> control_items_;

void publish_msg(rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher) {
  auto message = sensor_msgs::msg::JointState();
  message.header.stamp = rclcpp::Clock().now();
  message.name.resize(dxl_cnt);
  message.position.resize(dxl_cnt);
  message.velocity.resize(dxl_cnt);
  message.effort.resize(dxl_cnt);

  for (size_t i = 0; i < dxl_cnt; i++) {
    message.name[i] = "joint_" + std::to_string(joint_ids_[i]);
    message.position[i] = dynamixel_workbench_.convertValue2Radian(joint_ids_[i], joint_positions[i]);
    message.velocity[i] = dynamixel_workbench_.convertValue2Velocity(joint_ids_[i], joint_velocities[i]);
    message.effort[i] = 0.0;  // Effort data is not available
  }

  publisher->publish(message);
}

bool read() {
  const char* log = nullptr;
  bool result = true;

  // Read Velocities for all joints
  if (!dynamixel_workbench_.syncRead(0, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s : Joint velocity sync read failed", log);
    result = false;
  }
  else {
    if (!dynamixel_workbench_.getSyncReadData(
      0, joint_ids_, dxl_cnt,
      control_items_[kPresentVelocityItem]->address,
      control_items_[kPresentVelocityItem]->data_length, joint_velocities, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
        "%s : Joint velocity sync read data failed", log);
    }

    // Convert and print the velocity values
    for (size_t i = 0; i < dxl_cnt; i++) {
      double velocity_rad_s = dynamixel_workbench_.convertValue2Velocity(joint_ids_[i], joint_velocities[i]);

      // printf("Joint ID: %d, Velocity: %d (%f rad/s)\n",
      //   joint_ids_[i], joint_velocities[i], velocity_rad_s);
    }
  }

  // Read Positions for all joints
  if (!dynamixel_workbench_.syncRead(1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s : Joint position sync read failed", log);
    result = false;
  }
  else {
    if (!dynamixel_workbench_.getSyncReadData(
      1, joint_ids_, dxl_cnt,
      control_items_[kPresentPositionItem]->address,
      control_items_[kPresentPositionItem]->data_length, joint_positions, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
        "%s : Joint position sync read data failed", log);
    }

    // Convert and print the position values
    for (size_t i = 0; i < dxl_cnt; i++) {
      double position_rad = dynamixel_workbench_.convertValue2Radian(joint_ids_[i], joint_positions[i]);

      // printf("Joint ID: %d, Position: %d (%f rad)\n",
      //   joint_ids_[i], joint_positions[i], position_rad);
    }
  }

  publish_msg(rclcpp::Node::make_shared("dynamixel_sync_read")->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10));

  return result;
}

int main(int argc, char* argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamixel_sync_read");


  bool result = false;
  const char* log = nullptr;
  result = dynamixel_workbench_.init(port_name, baud_rate, &log);
  if (result == false) {
    printf("%s\n", log);
    return 0;
  }

  for (size_t i = 0; i < dxl_cnt; i++) {
    result = dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log);
    if (result == false) {
      printf("%s\n", log);
    }
    else {
      printf("id : %d model : %d\n", joint_ids_[i], model_number);
    }
  }

  const ControlItem* present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Control item %s not found.", kPresentPositionItem);
    return -1;
  }

  const ControlItem* present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Control item %s not found.", kPresentVelocityItem);
    return -1;
  }

  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentPositionItem] = present_position;

  // Sync Read Velocities
  if (!dynamixel_workbench_.addSyncReadHandler(
    control_items_[kPresentVelocityItem]->address, control_items_[kPresentVelocityItem]->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s : Velocity sync read handler failed", log);
    return -1;
  }

  // Sync Read Positions
  if (!dynamixel_workbench_.addSyncReadHandler(
    control_items_[kPresentPositionItem]->address, control_items_[kPresentPositionItem]->data_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s : Position sync read handler failed", log);
    return -1;
  }

  // Create timer that runs at 125Hz
  auto timer_callback = [&]() -> void {
    if (read()) {
      RCLCPP_DEBUG(node->get_logger(), "Sync read successful");
    }
    else {
      RCLCPP_WARN(node->get_logger(), "Sync read failed");
    }
    };

  // 125Hz = period of 8ms
  int frequency = 125;
  auto timer = node->create_wall_timer(std::chrono::milliseconds(1000 / frequency), timer_callback);

  RCLCPP_INFO(node->get_logger(), "Starting 125Hz sync read loop");

  // Keep the program alive to execute callbacks
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
