#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace altair_hardware
{
  constexpr const char* kDynamixelHardware = "DynamixelHardware";
  constexpr uint8_t kGoalPositionIndex = 0;
  constexpr uint8_t kReadIndex_MX64 = 0;
  constexpr uint8_t kReadIndex_MX28 = 1;
  constexpr const char* kGoalPositionItem = "Goal_Position";
  constexpr const char* kPresentPositionItem = "Present_Position";
  constexpr const char* kPresentVelocityItem = "Present_Velocity";
  constexpr const char* const kExtraJointParameters[] = {
    "Drive_Mode",
  };
  // "Profile_Velocity",
  // "Profile_Acceleration",
  // "Position_P_Gain",
  // "Position_I_Gain",
  // "Position_D_Gain"


  DynamixelHardware::DynamixelHardware()
    : rclcpp::Node("dynamixel_hardware"),
    logger_(rclcpp::get_logger("dynamixel_hardware"))
  {
  }

  DynamixelHardware::~DynamixelHardware()
  {
    if (rclcpp::ok()) {
      RCLCPP_INFO(logger_, "Shutting down ROS2 node...");
    }
  }

  CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo& info)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    joint_count_ = info_.joints.size();
    if (joint_count_ > MAX_JOINTS) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Too many joints: %zu (max: %zu)", joint_count_, MAX_JOINTS);
      return CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < joint_count_; i++) {
      joint_ids_[i] = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters.at("id")));
      joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].prev_command.position = joints_[i].command.position;
      target_positions_[i] = 0.0;
    }

    // IF tuning_mode ENABLED
    if (info_.hardware_parameters["tuning_mode"] == "True") {
      std::cout << "################ TUNING MODE ENABLED ################" << std::endl;
      tuning_mode_ = true;
      using namespace std::placeholders;

      std::string set_ping_srv_name = info_.hardware_parameters["set_ping_srv_name"];
      set_ping_srv_ = this->create_service<dynamixel_msgs::srv::SetPing>(
        set_ping_srv_name,
        std::bind(&DynamixelHardware::set_ping_srv_callback, this, _1, _2));

      std::string str_set_dxl_torque_id_srv_name =
        info_.hardware_parameters["set_dxl_torque_id_srv_name"];
      set_dxl_torque_id_srv_ = this->create_service<dynamixel_msgs::srv::SetTorque>(
        str_set_dxl_torque_id_srv_name,
        std::bind(&DynamixelHardware::set_dxl_torque_id_srv_callback, this, _1, _2));

      std::string set_init_pose_srv_name = info_.hardware_parameters["set_init_pose_srv_name"];
      set_init_pose_srv_ = this->create_service<std_srvs::srv::SetBool>(
        set_init_pose_srv_name,
        std::bind(&DynamixelHardware::set_init_pose_srv_callback, this, _1, _2));
    }
    // IF sim_mode ENABLED
    if (
      info_.hardware_parameters.find("sim_mode") != info_.hardware_parameters.end() &&
      info_.hardware_parameters.at("sim_mode") == "True")
    {
      sim_mode_ = true;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
      return CallbackReturn::SUCCESS;
    }

    auto portname = info_.hardware_parameters.at("portname");
    auto baudrate = std::stoi(info_.hardware_parameters.at("baudrate"));
    const char* log = nullptr;

    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "portname: %s", portname.c_str());
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baudrate: %d", baudrate);

    try {
      if (!dynamixel_workbench_.init(portname.c_str(), baudrate, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
    }
    catch (const std::exception& e) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Exception during initialization: %s", e.what());
      return CallbackReturn::ERROR;
    }

    for (uint i = 0; i < info_.joints.size(); ++i) {
      uint16_t model_number = 0;
      if (dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "PING: ID: %d Model: %s", joint_ids_[i], dynamixel_workbench_.getModelName(joint_ids_[i]));
      }
      else {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
    }

    // if (tuning_mode_ == true && sim_mode_ == false) {
    //   // set velocity profile to 5 and acceleration profile to 0
    //   for (uint i = 0; i < joint_ids_.size(); i++) {
    //     dynamixel_workbench_.jointMode(joint_ids_[i], 5, 0, &log);
    //   }
    // }

    enable_torque(false);
    set_control_mode(ControlMode::Position, true);
    set_joint_params();
    // enable_torque(true);

    const ControlItem* goal_position =
      dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
    if (goal_position == nullptr) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Control item %s not found.", kGoalPositionItem);
      return CallbackReturn::ERROR;
    }

    const ControlItem* present_position =
      dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
    if (present_position == nullptr) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Control item %s not found.", kPresentPositionItem);
      return CallbackReturn::ERROR;
    }

    const ControlItem* present_velocity =
      dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
    if (present_velocity == nullptr) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Control item %s not found.", kPresentVelocityItem);
      return CallbackReturn::ERROR;
    }

    control_items_[kGoalPositionItem] = goal_position;
    control_items_[kPresentVelocityItem] = present_velocity;
    control_items_[kPresentPositionItem] = present_position;

    if (!dynamixel_workbench_.addSyncWriteHandler(
      control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
      &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s : addSyncWrite handler failed", log);
      return CallbackReturn::ERROR;
    }

    uint16_t start_address = std::min(control_items_[kPresentVelocityItem]->address, control_items_[kPresentPositionItem]->address);

    uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
      control_items_[kPresentVelocityItem]->data_length + 4; // 4 atau 2?

    if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s : addSyncRead handler failed", log);
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Sync Read Handler Count: %d",
      dynamixel_workbench_.getTheNumberOfSyncReadHandler());

    mot_cont_ = 200;

    sync_read_error_flag_ = true;

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(joint_count_ * 2);  // Reserve space to avoid reallocations

    for (size_t i = 0; i < joint_count_; i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(joint_count_);  // Reserve space

    for (size_t i = 0; i < joint_count_; i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    }

    return command_interfaces;
  }

  CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
    for (size_t i = 0; i < joint_count_; i++) {
      if (sim_mode_ && std::isnan(joints_[i].state.position)) {
        joints_[i].state.position = 0.0;
        joints_[i].state.velocity = 0.0;
      }
    }
    while (sync_read_error_flag_ == true) {
      read(rclcpp::Time{}, rclcpp::Duration(0, 0));
    }
    enable_torque(true); // enable after getting correct reading
    reset_command();
    set_init_position();  // Now this will set target positions, not command positions
    init_position_ = true;
    // Initialize joint offsets
    set_joint_offset();
    write(rclcpp::Time{}, rclcpp::Duration(0, 0));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHardware::on_deactivate(
    const rclcpp_lifecycle::State& /* previous_state */)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHardware::on_error(
    const rclcpp_lifecycle::State& /* previous_state */)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "on_error");
    return CallbackReturn::SUCCESS;
  }

  return_type DynamixelHardware::read(
    const rclcpp::Time& /* time */,
    const rclcpp::Duration& /* period */)
  {
    rclcpp::spin_some(this->get_node_base_interface());

    if (sim_mode_) {
      return return_type::OK;
    }

    int32_t positions[MAX_JOINTS];
    int32_t velocities[MAX_JOINTS];
    const char* log = nullptr;

    if (!dynamixel_workbench_.syncRead(
      kReadIndex_MX64, joint_ids_.data(), joint_count_, &log))
    {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(kDynamixelHardware), *this->rclcpp::Node::get_clock(), 1000, "%s", log);
      return return_type::OK;
    }

    sync_read_error_flag_ = false;

    if (!dynamixel_workbench_.getSyncReadData(
      kReadIndex_MX64, joint_ids_.data(), joint_count_,
      control_items_[kPresentVelocityItem]->address,
      control_items_[kPresentVelocityItem]->data_length, velocities, &log))
    {
      return return_type::OK;
    }

    if (!dynamixel_workbench_.getSyncReadData(
      kReadIndex_MX64, joint_ids_.data(), joint_count_,
      control_items_[kPresentPositionItem]->address,
      control_items_[kPresentPositionItem]->data_length, positions, &log))
    {
      return return_type::OK;
    }

    for (size_t i = 0; i < joint_count_; i++) {
      joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(joint_ids_[i], positions[i]);
      joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(joint_ids_[i], velocities[i]);
    }

    return return_type::OK;
  }

  return_type DynamixelHardware::write(
    const rclcpp::Time& /* time */,
    const rclcpp::Duration& /* period */)
  {
    // Handle simulation mode
    if (sim_mode_) {
      for (size_t i = 0; i < joint_count_; i++) {
        joints_[i].prev_command.position = joints_[i].command.position;
        joints_[i].state.position = joints_[i].command.position;
      }
      return return_type::OK;
    }

    // Always write commands to servos
    write_to_servo();
    return return_type::OK;
  }

  return_type DynamixelHardware::enable_torque(const bool enabled)
  {
    const char* log = nullptr;

    if (enabled) {
      for (size_t i = 0; i < joint_count_; ++i) {
        if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      reset_command();
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
    }
    else {
      for (size_t i = 0; i < joint_count_; ++i) {
        if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
    }

    torque_enabled_ = enabled;
    return return_type::OK;
  }

  return_type DynamixelHardware::enable_torque(const bool enabled, const uint8_t* id_arr, size_t count)
  {
    const char* log = nullptr;

    if (enabled) {
      for (size_t i = 0; i < count; ++i) {
        if (!dynamixel_workbench_.torqueOn(id_arr[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
        RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "Torque enabled for ID: %d", id_arr[i]);
      }
      reset_command();
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled for specified IDs");
    }
    else {
      for (size_t i = 0; i < count; ++i) {
        if (!dynamixel_workbench_.torqueOff(id_arr[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
        RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "Torque disabled for ID: %d", id_arr[i]);
      }
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled for specified IDs");
    }

    torque_enabled_ = enabled;
    return return_type::OK;
  }

  return_type DynamixelHardware::set_control_mode(const ControlMode& mode, const bool force_set)
  {
    const char* log = nullptr;
    mode_changed_ = false;

    if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
      bool torque_enabled = torque_enabled_;
      if (torque_enabled) {
        enable_torque(false);
      }

      for (size_t i = 0; i < joint_count_; ++i) {
        if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");
      if (control_mode_ != ControlMode::Position) {
        mode_changed_ = true;
        control_mode_ = ControlMode::Position;
      }

      if (torque_enabled) {
        enable_torque(true);
      }
      return return_type::OK;
    }

    return return_type::OK;
  }

  return_type DynamixelHardware::reset_command()
  {
    for (size_t i = 0; i < joint_count_; i++) {
      joints_[i].command.position = joints_[i].state.position;
      joints_[i].prev_command.position = joints_[i].command.position;
    }

    return return_type::OK;
  }

  void DynamixelHardware::move_smoother(double& current, double target) {
    // Arguments: current: reference to current servo position, target: target servo position
    // Optional steps parameter to control smoothing rate (higher = smoother but slower)
    if (mot_cont_ < 1) {
      current = target;
    }
    else {
      current += (target - current) / mot_cont_;
    }
  }

  return_type DynamixelHardware::go_to_init()
  {
    for (size_t i = 0; i < joint_count_; ++i) {
      move_smoother(joints_[i].command.position, 0.0);
    }
    if (mot_cont_ > 0) --mot_cont_;
    else {
      init_position_ = false;
    }

    return return_type::OK;
  }

  return_type DynamixelHardware::set_init_position()
  {
    for (size_t i = 0; i < joint_count_; i++) {
      target_positions_[i] = 0.0;
    }

    return return_type::OK;
  }

  CallbackReturn DynamixelHardware::write_to_servo()
  {
    const char* log = nullptr;
    // Use member arrays directly instead of temporary stack arrays
    int32_t commands[MAX_JOINTS];

    if (init_position_) {
      for (size_t i = 0; i < joint_count_; i++) {
        // Inline the move_smoother logic to avoid function call overhead
        if (mot_cont_ < 1) {
          joints_[i].command.position = target_positions_[i];
        }
        else {
          joints_[i].command.position += (target_positions_[i] - joints_[i].command.position) / mot_cont_;
        }
      }

      if (mot_cont_ > 0) {
        --mot_cont_;
      }
      else {
        init_position_ = false;
        RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Initialization position reached");
      }
    }

    // Generate commands directly without copying IDs
    for (size_t i = 0; i < joint_count_; i++) {
      joints_[i].prev_command.position = joints_[i].command.position;
      commands[i] = dynamixel_workbench_.convertRadian2Value(
        joint_ids_[i], static_cast<float>(joints_[i].command.position + joint_offset_[i]));
    }

    // Use joint_ids_ array directly instead of copying to temporary array
    if (!dynamixel_workbench_.syncWrite(
      kGoalPositionIndex, joint_ids_.data(), joint_count_, commands, 1, &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHardware::set_joint_params()
  {
    const char* log = nullptr;
    for (size_t i = 0; i < joint_count_; ++i) {
      for (auto paramName : kExtraJointParameters) {
        if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
          auto value = std::stoi(info_.joints[i].parameters.at(paramName));
          if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
            RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
            return CallbackReturn::ERROR;
          }
          RCLCPP_INFO(
            rclcpp::get_logger(
              kDynamixelHardware), "%s set to %d for joint %zu", paramName, value, i);
        }
      }
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHardware::set_joint_offset() {
    for (size_t i = 0; i < joint_count_; ++i) {
      joint_offset_[i] = std::stod(info_.joints[i].parameters.at("offset"));
    }
    return CallbackReturn::SUCCESS;
  }

  void DynamixelHardware::set_dxl_torque_id_srv_callback(
    const std::shared_ptr<dynamixel_msgs::srv::SetTorque::Request> request,
    std::shared_ptr<dynamixel_msgs::srv::SetTorque::Response> response)
  {
    const auto& ids = request->id;
    bool torque_enable = request->enable;

    for (const auto& id : ids) {
      RCLCPP_INFO(logger_, "ID: %d, Torque: %s", id, torque_enable ? "ON" : "OFF");
    }

    if (enable_torque(torque_enable, ids.data(), ids.size()) != return_type::OK) {
      response->result = false;
      return;
    }

    response->result = true;
  }

  void DynamixelHardware::set_ping_srv_callback(
    const std::shared_ptr<dynamixel_msgs::srv::SetPing::Request> request,
    std::shared_ptr<dynamixel_msgs::srv::SetPing::Response> response)
  {
    RCLCPP_INFO(logger_, "Pinging servos...");
    const char* log = nullptr;
    const auto& ids = request->id;
    std::vector<uint8_t> results;
    results.reserve(ids.size());  // Reserve space to avoid reallocations

    for (const auto& id : ids) {
      uint16_t model_number = 0;
      if (dynamixel_workbench_.ping(id, &model_number, &log)) {
        results.push_back(id);
        RCLCPP_INFO(logger_, "PING: ID: %d Model: %s Model_Number: %d", id, dynamixel_workbench_.getModelName(id), model_number);
      }
      else {
        RCLCPP_ERROR(logger_, "%s", log);
      }
    }
    response->result = results;
  }

  void DynamixelHardware::set_init_pose_srv_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    RCLCPP_INFO(logger_, "Setting initialization pose...");
    if (request->data)
    {
      init_position_ = true;
      auto result = set_init_position();
      response->success = (result == return_type::OK);
      response->message = response->success ? "go to zero init in progress." : "Failed to set initialization pose.";
      return;
    }
    else
    {
      response->success = false;
      response->message = "Request data is false. Go to zero init is not set.";
    }
  }

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(altair_hardware::DynamixelHardware, hardware_interface::SystemInterface)

//---------------------------------------------------------
// MX Protocol 2 servos - (num == MX_28_2)
//---------------------------------------------------------
// static const ControlItem items_MX2[]{
//   {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
//   {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
//   {s_ID, 7, sizeof(s_ID) - 1, 1},
//   {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
//   {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
//   {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
//   {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
//   {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
//   {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
//   {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
//   {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
//   {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
//   {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
//   {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
//   {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
//   {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
//   {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
//   {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
//   {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
//   {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

//   {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
//   {s_LED, 65, sizeof(s_LED) - 1, 1},
//   {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
//   {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
//   {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
//   {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
//   {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
//   {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
//   {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
//   {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
//   {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
//   {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
//   {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
//   {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
//   {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
//   {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
//   {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
//   {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
//   {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
//   {s_Moving, 122, sizeof(s_Moving) - 1, 1},
//   {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
//   {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
//   {s_Present_Load, 126, sizeof(s_Present_Load) - 1, 2},
//   {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
//   {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
//   {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
//   {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
//   {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
//   {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};

//---------------------------------------------------------
// EXT MX Protocol 2 Servos - (num == MX_64_2 || num == MX_106_2)
//---------------------------------------------------------
// static const ControlItem items_EXTMX2[]{
//   {s_Model_Number, 0, sizeof(s_Model_Number) - 1, 2},
//   {s_Firmware_Version, 6, sizeof(s_Firmware_Version) - 1, 1},
//   {s_ID, 7, sizeof(s_ID) - 1, 1},
//   {s_Baud_Rate, 8, sizeof(s_Baud_Rate) - 1, 1},
//   {s_Return_Delay_Time, 9, sizeof(s_Return_Delay_Time) - 1, 1},
//   {s_Drive_Mode, 10, sizeof(s_Drive_Mode) - 1, 1},
//   {s_Operating_Mode, 11, sizeof(s_Operating_Mode) - 1, 1},
//   {s_Secondary_ID, 12, sizeof(s_Secondary_ID) - 1, 1},
//   {s_Protocol_Version, 13, sizeof(s_Protocol_Version) - 1, 1},
//   {s_Homing_Offset, 20, sizeof(s_Homing_Offset) - 1, 4},
//   {s_Moving_Threshold, 24, sizeof(s_Moving_Threshold) - 1, 4},
//   {s_Temperature_Limit, 31, sizeof(s_Temperature_Limit) - 1, 1},
//   {s_Max_Voltage_Limit, 32, sizeof(s_Max_Voltage_Limit) - 1, 2},
//   {s_Min_Voltage_Limit, 34, sizeof(s_Min_Voltage_Limit) - 1, 2},
//   {s_PWM_Limit, 36, sizeof(s_PWM_Limit) - 1, 2},
//   {s_Current_Limit, 38, sizeof(s_Current_Limit) - 1, 2},
//   {s_Acceleration_Limit, 40, sizeof(s_Acceleration_Limit) - 1, 4},
//   {s_Velocity_Limit, 44, sizeof(s_Velocity_Limit) - 1, 4},
//   {s_Max_Position_Limit, 48, sizeof(s_Max_Position_Limit) - 1, 4},
//   {s_Min_Position_Limit, 52, sizeof(s_Min_Position_Limit) - 1, 4},
//   {s_Shutdown, 63, sizeof(s_Shutdown) - 1, 1},

//   {s_Torque_Enable, 64, sizeof(s_Torque_Enable) - 1, 1},
//   {s_LED, 65, sizeof(s_LED) - 1, 1},
//   {s_Status_Return_Level, 68, sizeof(s_Status_Return_Level) - 1, 1},
//   {s_Registered_Instruction, 69, sizeof(s_Registered_Instruction) - 1, 1},
//   {s_Hardware_Error_Status, 70, sizeof(s_Hardware_Error_Status) - 1, 1},
//   {s_Velocity_I_Gain, 76, sizeof(s_Velocity_I_Gain) - 1, 2},
//   {s_Velocity_P_Gain, 78, sizeof(s_Velocity_P_Gain) - 1, 2},
//   {s_Position_D_Gain, 80, sizeof(s_Position_D_Gain) - 1, 2},
//   {s_Position_I_Gain, 82, sizeof(s_Position_I_Gain) - 1, 2},
//   {s_Position_P_Gain, 84, sizeof(s_Position_P_Gain) - 1, 2},
//   {s_Feedforward_2nd_Gain, 88, sizeof(s_Feedforward_2nd_Gain) - 1, 2},
//   {s_Feedforward_1st_Gain, 90, sizeof(s_Feedforward_1st_Gain) - 1, 2},
//   {s_Bus_Watchdog, 98, sizeof(s_Bus_Watchdog) - 1, 1},
//   {s_Goal_PWM, 100, sizeof(s_Goal_PWM) - 1, 2},
//   {s_Goal_Current, 102, sizeof(s_Goal_Current) - 1, 2},
//   {s_Goal_Velocity, 104, sizeof(s_Goal_Velocity) - 1, 4},
//   {s_Profile_Acceleration, 108, sizeof(s_Profile_Acceleration) - 1, 4},
//   {s_Profile_Velocity, 112, sizeof(s_Profile_Velocity) - 1, 4},
//   {s_Goal_Position, 116, sizeof(s_Goal_Position) - 1, 4},
//   {s_Realtime_Tick, 120, sizeof(s_Realtime_Tick) - 1, 2},
//   {s_Moving, 122, sizeof(s_Moving) - 1, 1},
//   {s_Moving_Status, 123, sizeof(s_Moving_Status) - 1, 1},
//   {s_Present_PWM, 124, sizeof(s_Present_PWM) - 1, 2},
//   {s_Present_Current, 126, sizeof(s_Present_Current) - 1, 2},
//   {s_Present_Velocity, 128, sizeof(s_Present_Velocity) - 1, 4},
//   {s_Present_Position, 132, sizeof(s_Present_Position) - 1, 4},
//   {s_Velocity_Trajectory, 136, sizeof(s_Velocity_Trajectory) - 1, 4},
//   {s_Position_Trajectory, 140, sizeof(s_Position_Trajectory) - 1, 4},
//   {s_Present_Input_Voltage, 144, sizeof(s_Present_Input_Voltage) - 1, 2},
//   {s_Present_Temperature, 146, sizeof(s_Present_Temperature) - 1, 1}};
