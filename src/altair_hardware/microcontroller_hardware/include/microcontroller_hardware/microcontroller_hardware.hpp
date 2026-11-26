#ifndef ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
#define ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rot_conv/rot_conv/rot_conv.h"
#include "serialib.hpp"

using namespace std;

namespace altair_hardware {

  class MicrocontrollerHardware : public hardware_interface::SensorInterface, rclcpp::Node {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MicrocontrollerHardware);

    MicrocontrollerHardware();
    ~MicrocontrollerHardware();

    void parse_imu_data(const char* raw_data);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:
    std::unique_ptr<serialib> serial_port_;
    std::atomic<bool> stop_thread;

    const char* portname_;
    int baudrate_;

    std::array<double, 4> orientation_;
    std::array<double, 3> angular_velocity_;
    std::array<double, 3> linear_acceleration_;
    double button_msg_;
    int bytesRead; 

    // USB reconnection parameters
    int max_reconnection_attempts_;
    int reconnection_delay_ms_;

    // Attempts to reconnect to the USB device
    bool attempt_reconnection();

    rot_conv::Quat imu_quaternion;
  };

}  // namespace altair_hardware

#endif  // ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
