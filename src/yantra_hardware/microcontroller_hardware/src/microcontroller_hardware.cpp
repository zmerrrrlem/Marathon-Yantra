#include "microcontroller_hardware/microcontroller_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sys/select.h>
#include <mutex>



#define serial_port "/dev/stm32"
#define BAUDRATE B1000000


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace yantra_hardware
{

  MicrocontrollerHardware::MicrocontrollerHardware()
    : rclcpp::Node("MicrocontrollerHardware"), stop_thread(false), max_reconnection_attempts_(5), 
      reconnection_delay_ms_(1000)
  {
  }
  MicrocontrollerHardware::~MicrocontrollerHardware()
  {
    stop_thread = true;
    if (serial_port_) {
      serial_port_->closeDevice();
    }
  }

  hardware_interface::CallbackReturn MicrocontrollerHardware::on_init(const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Takes parameter defined in urdf file. Use xacro variable to pass parameter from launch file
    portname_ = info_.hardware_parameters["portname"].c_str();
    baudrate_ = stoi(info_.hardware_parameters["baudrate"]);

    serial_port_ = std::make_unique<serialib>();

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {


    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MicrocontrollerHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces_conf;

    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &orientation_[0]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &orientation_[1]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &orientation_[2]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &orientation_[3]));

    // Add state interfaces for angular velocity
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &angular_velocity_[0]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &angular_velocity_[1]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &angular_velocity_[2]));

    // Add state interfaces for linear acceleration
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &linear_acceleration_[0]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &linear_acceleration_[1]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &linear_acceleration_[2]));

    // Button State Interface
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name, &button_msg_));

    std::cout << info_.sensors[1].state_interfaces[0].name << " ----------------" << endl;

    return state_interfaces_conf;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    if (!serial_port_ || serial_port_->openDevice(portname_, baudrate_) != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Failed to open serial port on activation");
      if (!attempt_reconnection()) {
        RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Failed to reconnect during activation");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {

    if (serial_port_) {
      serial_port_->closeDevice();
    }
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MicrocontrollerHardware::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Hardware entered error state, attempting to recover");
    if (attempt_reconnection()) {
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  void MicrocontrollerHardware::parse_imu_data(const char* raw_data) {
    sscanf(raw_data, "$U%lfG%lfM%lfD%lfI%lfY%lfP%lfC%lfT%lfB%d",
      &orientation_[0], &orientation_[1], &orientation_[2], 
      &angular_velocity_[0], &angular_velocity_[1], &angular_velocity_[2],
      &linear_acceleration_[0], &linear_acceleration_[1], &linear_acceleration_[2], 
      &button_msg_);
    
    imu_quaternion = rot_conv::QuatFromEuler(orientation_[2], orientation_[1], orientation_[0]);
    orientation_[0] = imu_quaternion.x();
    orientation_[1] = imu_quaternion.y();
    orientation_[2] = imu_quaternion.z();
    orientation_[3] = imu_quaternion.w();
  }

  hardware_interface::return_type MicrocontrollerHardware::read(const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
  {
    char buffer[256] = { 0 };
    bytesRead = serial_port_->readString(buffer, '\n', sizeof(buffer) - 1, 100);
    
    if (bytesRead > 0) {
      parse_imu_data(buffer);
    }
    else if (bytesRead < 1) {
      RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Serial device disconnected. Attempting to reconnect...");
      if (!attempt_reconnection()) {
        RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Reconnection attempts failed");
        return hardware_interface::return_type::ERROR;
      }
    }

    return hardware_interface::return_type::OK;
  }

  bool MicrocontrollerHardware::attempt_reconnection() {
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Attempting to reconnect to USB device...");
    
    // Close the device first if it's open
    if (serial_port_) {
      serial_port_->closeDevice();
    }
    
    // Try to reconnect multiple times
    for (int attempt = 1; attempt <= max_reconnection_attempts_; ++attempt) {
      RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), 
                 "Reconnection attempt %d of %d", attempt, max_reconnection_attempts_);
      
      if (serial_port_->openDevice(portname_, baudrate_) == 1) {
        RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully reconnected to USB device");
        return true;
      }
      
      // Wait before trying again
      std::this_thread::sleep_for(std::chrono::milliseconds(reconnection_delay_ms_));
    }
    
    RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), 
                "Failed to reconnect after %d attempts", max_reconnection_attempts_);
    return false;
  }

}  // namespace yantra_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(yantra_hardware::MicrocontrollerHardware, hardware_interface::SensorInterface)
