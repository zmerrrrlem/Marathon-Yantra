#include "microcontroller_hardware/microcontroller_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <regex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace yantra_hardware
{
  hardware_interface::CallbackReturn MicrocontrollerHardware::on_init(const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Takes parameter defined in urdf file. Use xacro variable to pass parameter from launch file
    portname_ = info_.hardware_parameters["portname"];
    baudrate_ = stoi(info_.hardware_parameters["baudrate"]);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully configured!");
    orientation_.fill(0.0);
    angular_velocity_.fill(0.0);
    linear_acceleration_.fill(0.0);

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
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Activating ...please wait...");

    serial_read_.Open(portname_);
    serial_read_.SetBaudRate(BaudRate::BAUD_115200);
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    serial_read_.Close();
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully deactivated!");
    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MicrocontrollerHardware::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    serial_read_.Close();
    serial_read_.Open(portname_);
    serial_read_.SetBaudRate(BaudRate::BAUD_115200);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type MicrocontrollerHardware::read(const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
  {
    serial_read_.ReadLine(read_buffer, '\n');

    // Correct regex pattern: Matches character keys (U, G, M...) followed by a float value (handles negatives)
    std::regex pattern(R"(([UGMPCTDIYB])(-?\d+(\.\d+)?))");
    std::sregex_iterator it(read_buffer.begin(), read_buffer.end(), pattern);
    std::sregex_iterator end;

    while (it != end) {
      char type = (*it)[1].str()[0];   // Extract key (U, G, M, etc.)
      double value = std::stod((*it)[2].str());  // Extract value as double
      ++it;  // Move to next match

      switch (type) {
      case 'U':
        roll = -1 * value * DEGREE2RADIAN;
        imu_msg_.orientation.x = roll;
        orientation_[0] = roll;
        break;
      case 'G':
        pitch = value * DEGREE2RADIAN;
        imu_msg_.orientation.y = pitch;
        orientation_[1] = pitch;
        break;
      case 'M':
        yaw = -1 * value * DEGREE2RADIAN;
        imu_msg_.orientation.z = yaw;
        orientation_[2] = yaw;
        break;
      case 'D':
        gyroRoll = value * DEGREE2RADIAN;
        imu_msg_.angular_velocity.x = gyroRoll;
        angular_velocity_[0] = gyroRoll;
        break;
      case 'I':
        gyroPitch = -1 * value * DEGREE2RADIAN;
        imu_msg_.angular_velocity.y = gyroPitch;
        angular_velocity_[1] = gyroPitch;
        break;
      case 'Y':
        gyroYaw = value * DEGREE2RADIAN;
        imu_msg_.angular_velocity.z = gyroYaw;
        angular_velocity_[2] = gyroYaw;
        break;
      case 'P':
        accelX = value;
        imu_msg_.linear_acceleration.x = accelX;
        linear_acceleration_[0] = accelX;
        break;
      case 'C':
        accelY = value;
        imu_msg_.linear_acceleration.y = accelY;
        linear_acceleration_[1] = accelY;
        break;
      case 'T':
        accelZ = value;
        imu_msg_.linear_acceleration.z = accelZ;
        linear_acceleration_[2] = accelZ;
        data_ready = true;
        break;
      case 'B':
        button_msg_ = static_cast<int>(value);
        break;
      }
    }

    return hardware_interface::return_type::OK;
  }

}  // namespace yantra_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(yantra_hardware::MicrocontrollerHardware, hardware_interface::SensorInterface)
