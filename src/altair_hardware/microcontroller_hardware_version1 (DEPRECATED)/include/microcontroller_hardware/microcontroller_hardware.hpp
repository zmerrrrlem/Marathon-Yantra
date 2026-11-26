#ifndef ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
#define ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <string>
#include <iostream>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/char.hpp"

#include <fstream>
#include <cstdlib>
#include <linux/serial.h>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
using namespace LibSerial;
using namespace std;

namespace altair_hardware {
class MicrocontrollerHardware : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MicrocontrollerHardware);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  SerialPort serial_read_;

private:
  rclcpp::Node::SharedPtr nh_;
  sensor_msgs::msg::Imu imu_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr button_pub_;

  std::string portname_;
  int baudrate_;
  int waitTimeout;
  int timeOut_ctr = 0;
  bool broadcast = broadcast;
  bool reconnect = false;
  bool dev_connected = false;

  bool data_ready = false;
  float roll, pitch, yaw, gyroRoll, gyroPitch, gyroYaw, accelX, accelY, accelZ, temp_data;
  std::string read_buffer;
  char buffer;

  float button;
  char type;
  double value;

  std::vector<double> hw_states_;
  std::array<double, 4> orientation_;
  std::array<double, 3> angular_velocity_;
  std::array<double, 3> linear_acceleration_;
  double button_msg_;

#define DEGREE2RADIAN 0.01745;
};

}  // namespace altair_hardware

#endif  // ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
