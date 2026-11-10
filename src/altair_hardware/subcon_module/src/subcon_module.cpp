#include "subcon_module/subcon_module.hpp"

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

#define DEGREE2RADIAN 0.01745

using namespace robotis_op;

SubconModule::SubconModule()
  : Node("subcon_module")
  , control_cycle_msec_(8)
  , DEBUG_PRINT(false)
  , stop_thread(false)
  , max_reconnection_attempts_(5)
  , reconnection_delay_ms_(1000)
{
  module_name_ = "subcon_module";  // set unique module name

  result_["roll"] = 0.0;
  result_["pitch"] = 0.0;
  result_["yaw"] = 0.0;

  result_["gyro_x"] = 0.0;
  result_["gyro_y"] = 0.0;
  result_["gyro_z"] = 0.0;

  result_["acc_x"] = 0.0;
  result_["acc_y"] = 0.0;
  result_["acc_z"] = 0.0;

  result_["button"] = 0;

  previous_result_["gyro_x"] = 0.0;
  previous_result_["gyro_y"] = 0.0;
  previous_result_["gyro_z"] = 0.0;

  previous_result_["gyro_x_prev"] = 0.0;
  previous_result_["gyro_y_prev"] = 0.0;
  previous_result_["gyro_z_prev"] = 0.0;

  previous_result_["acc_x"] = 0.0;
  previous_result_["acc_y"] = 0.0;
  previous_result_["acc_z"] = 0.0;

  last_msg_time_ = rclcpp::Clock().now();
}

SubconModule::~SubconModule()
{
  stop_thread = true;
  if (serial_port_)
  {
    serial_port_->closeDevice();
  }
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();
}

void SubconModule::initialize(const int control_cycle_msec, robotis_framework::Robot* robot)
{
  portname_ = "/dev/ttyACM0";
  baudrate_ = 115200;

  serial_port_ = std::make_unique<serialib>();

  serial_port_.setDTR();
  serial_port_.setRTS();

  if (!serial_port_ || serial_port_->openDevice(portname_, baudrate_) != 1)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port on activation");
    if (!attempt_reconnection())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to reconnect during activation");
      return;
    }
  }

  control_cycle_msec_ = control_cycle_msec;
  queue_thread_ = std::thread(&SubconModule::queueThread, this);
}

void SubconModule::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  /* publisher */
  status_msg_pub_ = this->create_publisher<robotis_controller_msgs::msg::StatusMsg>("altair/status", 1);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("altair/imu", 1);
  button_pub_ = this->create_publisher<std_msgs::msg::Int16>("altair/button", 1);

  rclcpp::WallRate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void SubconModule::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                           std::map<std::string, robotis_framework::Sensor*> sensors)
{
  if (sensors["subcon"] == NULL)
    return;

  auto start = std::chrono::steady_clock::now();

  char buffer[256] = { 0 };
  bytesRead = serial_port_->readString(buffer, '\n', sizeof(buffer) - 1, 100);

  if (bytesRead > 0)
  {
    sscanf(buffer, "$U%lfG%lfM%lfD%lfI%lfY%lfP%lfC%lfT%lfB%d", &orientation_[0], &orientation_[1], &orientation_[2],
           &angular_velocity_[0], &angular_velocity_[1], &angular_velocity_[2], &linear_acceleration_[0],
           &linear_acceleration_[1], &linear_acceleration_[2], &button_msg_);
  }
  else if (bytesRead < 1)
  {
    RCLCPP_ERROR(this->get_logger(), "Serial device disconnected. Attempting to reconnect...");
    if (!attempt_reconnection())
    {
      RCLCPP_ERROR(this->get_logger(), "Reconnection attempts failed");
      return;
    }
  }

  result_["roll"] = orientation_[0];
  result_["pitch"] = orientation_[1];
  result_["yaw"] = orientation_[2];

  result_["gyro_x"] = lowPassFilter(0.4, angular_velocity_[0] * DEGREE2RADIAN, previous_result_["gyro_x"]);
  result_["gyro_y"] = lowPassFilter(0.4, angular_velocity_[1] * DEGREE2RADIAN, previous_result_["gyro_y"]);
  result_["gyro_z"] = lowPassFilter(0.4, angular_velocity_[2] * DEGREE2RADIAN, previous_result_["gyro_z"]);

  // align axis of Accelerometer to robot and
  result_["acc_x"] = lowPassFilter(0.4, linear_acceleration_[0], previous_result_["acc_x"]);
  result_["acc_y"] = lowPassFilter(0.4, linear_acceleration_[1], previous_result_["acc_y"]);
  result_["acc_z"] = lowPassFilter(0.4, linear_acceleration_[2], previous_result_["acc_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Euler ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %lf, %lf, %lf", orientation_[0], orientation_[1],
                         orientation_[2]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Gyro ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", angular_velocity_[0], angular_velocity_[1], angular_velocity_[2]);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f", result_["gyro_x"], result_["gyro_y"],
                         result_["gyro_z"]);

  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, " ======================= Acc ======================== ");
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Raw : %d, %d, %d", linear_acceleration_[0], linear_acceleration_[1], linear_acceleration_[2]);
  RCLCPP_INFO_EXPRESSION(this->get_logger(), DEBUG_PRINT, "Filtered : %f, %f, %f", result_["acc_x"], result_["acc_y"],
                         result_["acc_z"]);

  rclcpp::Time update_time(sensors["subcon"]->sensor_state_->update_time_stamp_.sec_,
                           sensors["subcon"]->sensor_state_->update_time_stamp_.nsec_);
  rclcpp::Duration update_duration = rclcpp::Clock().now() - update_time;

  result_["button"] = static_cast<int>(button_msg_);

  publishIMU();
  publishButton();

  previous_result_["gyro_x_prev"] = result_["gyro_x"];
  previous_result_["gyro_y_prev"] = result_["gyro_y"];
  previous_result_["gyro_z_prev"] = result_["gyro_z"];

  auto duration = std::chrono::steady_clock::now() - start;
  if (duration > std::chrono::milliseconds(control_cycle_msec_ * 0.5)) RCLCPP_WARN(get_logger(), "Serial read too slow!");

}

void SubconModule::publishIMU()
{
  // fusion imu data
  imu_msg_.header.stamp = this->get_clock()->now();
  imu_msg_.header.frame_id = "body_link";
  double filter_alpha = 0.4;

  Eigen::Quaterniond orientation =
      robotis_framework::convertRPYToQuaternion(result_["roll"], result_["pitch"], result_["yaw"]);

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();

  // in rad/s
  long int _value = 0;
  int _arrd_length = 2;
  imu_msg_.angular_velocity.x = result_["gyro_x"];
  imu_msg_.angular_velocity.y = result_["gyro_y"];
  imu_msg_.angular_velocity.z = result_["gyro_z"];

  // in m/s^2
  imu_msg_.linear_acceleration.x = result_["acc_x"] * G_ACC;
  imu_msg_.linear_acceleration.y = result_["acc_y"] * G_ACC;
  imu_msg_.linear_acceleration.z = result_["acc_z"] * G_ACC;

  imu_pub_->publish(imu_msg_);
}

void SubconModule::publishButton()
{
  std_msgs::msg::Int16 button_msg;
  button_msg.data = button_msg_;

  button_pub_->publish(button_msg);
}

double SubconModule::lowPassFilter(double alpha, double x_new, double& x_old)
{
  double filtered_value = alpha * x_new + (1.0 - alpha) * x_old;
  x_old = filtered_value;

  return filtered_value;
}

bool SubconModule::attempt_reconnection()
{
  RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to USB device...");

  // Close the device first if it's open
  if (serial_port_)
  {
    serial_port_->closeDevice();
  }

  // Try to reconnect multiple times
  for (int attempt = 1; attempt <= max_reconnection_attempts_; ++attempt)
  {
    RCLCPP_INFO(this->get_logger(), "Reconnection attempt %d of %d", attempt, max_reconnection_attempts_);

    if (serial_port_->openDevice(portname_, baudrate_) == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully reconnected to USB device");
      return true;
    }

    // Wait before trying again
    std::this_thread::sleep_for(std::chrono::milliseconds(reconnection_delay_ms_));
  }

  RCLCPP_ERROR(this->get_logger(), "Failed to reconnect after %d attempts", max_reconnection_attempts_);
  return false;
}