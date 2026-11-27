#include "microcontroller_hardware_node/microcontroller_hardware.hpp"

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
#include "rclcpp/rclcpp.hpp"

#include "rot_conv/rot_conv/rot_conv.h"

#define serial_port "/dev/stm32"
#define BAUDRATE B1000000

namespace yantra_hardware
{

  MicrocontrollerHardware::MicrocontrollerHardware()
    : rclcpp::Node("MicrocontrollerHardware"), max_reconnection_attempts_(5), max_errors_before_reconnect_(5),
    reconnection_delay_ms_(1000)
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_sensor_broadcaster/imu", 10);
    button_pub_ = this->create_publisher<std_msgs::msg::Int8>("button_state", 1);
  }

  MicrocontrollerHardware::~MicrocontrollerHardware()
  {
    if (port_) {
      port_->closeDevice();
    }
  }

  bool MicrocontrollerHardware::on_init()
  {
    std::string portname_params = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    portname_ = portname_params.c_str();
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    RCLCPP_INFO(this->get_logger(), "Using port: %s at baudrate: %d", portname_, baudrate_);

    port_ = std::make_unique<serialib>();
    return 1;
  }

  bool MicrocontrollerHardware::on_activate()
  {
    if (!port_ || port_->openDevice(portname_, baudrate_) != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Failed to open serial port on activation");
      if (attempt_reconnection()) {
        RCLCPP_WARN(rclcpp::get_logger("MicrocontrollerHardware"), "Re-Run the node to retry.");
        return 0;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully activated!");
    return 1;
  }

  bool MicrocontrollerHardware::on_deactivate()
  {
    if (port_) {
      port_->closeDevice();
    }
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully deactivated!");

    return 1;
  }

  bool MicrocontrollerHardware::attempt_reconnection() {
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Attempting to reconnect to USB device...");

    // if (port_->isDeviceOpen()) {
    //   port_->closeDevice();
    // }

    // while (true) {
    //   int ret = port_->openDevice(portname_, baudrate_);
    //   if (ret == 1) {
    //     std::cout << "✅ Connected to " << portname_ << " at " << baudrate_ << " baud\n";
    //     return true;
    //   }
    //   else {
    //     std::cerr << "⚠️  Failed to open " << portname_
    //       << " (error=" << ret << "). Retrying...\n";
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //   }
    // }

    RCLCPP_WARN(rclcpp::get_logger("MicrocontrollerHardware"),
      "Make sure the device is connected and accessible.");
    return true;
  }

  bool MicrocontrollerHardware::wait_for_sync() {
    uint8_t byte;
    bool firstFF = false;

    while (true) {
      if (port_->readBytes(&byte, 1, 10, 500) != 1) {
        return false;
      }
      if (byte == 0xFF) {
        if (firstFF) {
          return true;
        }
        else {
          firstFF = true;
        }
      }
      else {
        firstFF = false;
      }
    }
  }

  bool MicrocontrollerHardware::read()
  {
    SensorPacket packet;

    // Read sync bytes (0xFF 0xFF)
    uint8_t sync[2];
    if (!wait_for_sync()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("MicrocontrollerHardware"), *get_clock(), 3000, "Waiting for sync...");
      return 0;
    }

    // Read length byte
    uint8_t length;
    if (port_->readBytes(&length, 1, 10, 500) != 1 || length != sizeof(SensorPacket) + 1) {
      RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Failed to read length %d byte", length);
      // error_count_++;
      return 0;
    }

    uint8_t buffer[length];
    if (port_->readBytes(buffer, length, 10, 500) != length) {
      RCLCPP_ERROR(rclcpp::get_logger("MicrocontrollerHardware"), "Failed to read complete packet");
      // error_count_++;
      return 0;
    }

    // Verify checksum
    uint8_t checksum = 0;
    for (int i = 0; i < length - 1; i++) {
      checksum += buffer[i];
    }
    checksum = (~checksum) & 0xFF;

    if (checksum != buffer[length - 1]) {
      RCLCPP_WARN(rclcpp::get_logger("MicrocontrollerHardware"), "Checksum mismatch");
      // error_count_++;
      return 0;
    }

    std::memcpy(&packet, buffer, sizeof(SensorPacket));

    rot_conv::Quat conv = rot_conv::QuatFromEuler(packet.yaw, packet.pitch, packet.roll); // Format: (w,x,y,z)

    // Parse the data
    orientation_[0] = conv.x();
    orientation_[1] = conv.y();
    orientation_[2] = conv.z();
    orientation_[3] = conv.w();
    angular_velocity_[0] = packet.gyroX;
    angular_velocity_[1] = packet.gyroY;
    angular_velocity_[2] = packet.gyroZ;
    linear_acceleration_[0] = packet.accelX;
    linear_acceleration_[1] = packet.accelY;
    linear_acceleration_[2] = packet.accelZ;
    button_msg_ = packet.buttonState;

    publish_msg();

    return 1;
  }

  void MicrocontrollerHardware::publish_msg() {
    sensor_msgs::msg::Imu msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    msg.orientation.x = orientation_[0];
    msg.orientation.y = orientation_[1];
    msg.orientation.z = orientation_[2];
    msg.orientation.w = orientation_[3];
    // Angular velocity
    msg.angular_velocity.x = angular_velocity_[0];
    msg.angular_velocity.y = angular_velocity_[1];
    msg.angular_velocity.z = angular_velocity_[2];
    // Linear acceleration
    msg.linear_acceleration.x = linear_acceleration_[0];
    msg.linear_acceleration.y = linear_acceleration_[1];
    msg.linear_acceleration.z = linear_acceleration_[2];
    imu_pub_->publish(msg);

    std_msgs::msg::Int8 button_msg;
    button_msg.data = static_cast<int8_t>(button_msg_);
    button_pub_->publish(button_msg);
  }

}  // namespace yantra_hardware

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<yantra_hardware::MicrocontrollerHardware>();

  // rclcpp::experimental::executors::EventsExecutor exec;
  rclcpp::executors::SingleThreadedExecutor exec;

  node->on_init();
  if (!node->on_activate()) {
    rclcpp::shutdown();
    return 0;
  }

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 100.0);
  rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void {
    node->read();
    });
  exec.add_node(node);
  exec.spin();
  node->on_deactivate();
  rclcpp::shutdown();
  return 0;
}
