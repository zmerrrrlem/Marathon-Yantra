#ifndef ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
#define ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int8.hpp"
#include "rclcpp/macros.hpp"
#include "serialib.hpp"

using namespace std;

namespace altair_hardware {
  struct __attribute__((packed)) SensorPacket {
    float x;
    float y;
    float z;
    float w;
    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;
    float buttonState;
  };

  class MicrocontrollerHardware : public rclcpp::Node {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MicrocontrollerHardware);

    MicrocontrollerHardware();
    ~MicrocontrollerHardware();

    bool on_init();
    bool on_configure();
    void publish_msg();
    bool on_activate();
    bool on_deactivate();
    bool on_error();
    bool read();
    bool attempt_reconnection();
    void reset_error_count();
    bool wait_for_sync();

  private:
    std::unique_ptr<serialib> port_;
    const char* portname_;
    int baudrate_;
    std::array<double, 4> orientation_;
    std::array<double, 3> angular_velocity_;
    std::array<double, 3> linear_acceleration_;
    double button_msg_;
    int bytesRead;
    int max_reconnection_attempts_;
    int reconnection_delay_ms_;
    uint8_t error_count_;
    int max_errors_before_reconnect_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr button_pub_;

  };

}  // namespace altair_hardware

#endif  // ALTAIR_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
