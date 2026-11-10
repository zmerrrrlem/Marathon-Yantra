#ifndef SUBCON_MODULE_HPP_
#define SUBCON_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_framework_common/sensor_module.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_linear_algebra.h"

#include "serialib.hpp"

namespace robotis_op
{

class SubconModule : public robotis_framework::SensorModule,
                     public robotis_framework::Singleton<SubconModule>,
                     public rclcpp::Node
{
public:
  SubconModule();
  virtual ~SubconModule();

  /* ROS Topic Callback Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot);
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, robotis_framework::Sensor*> sensors);

private:
  std::unique_ptr<serialib> serial_port_;
  const char* portname_;
  int baudrate_;

  std::array<double, 4> orientation_;
  std::array<double, 3> angular_velocity_;
  std::array<double, 3> linear_acceleration_;
  int bytesRead;
  
  // USB reconnection parameters
  int max_reconnection_attempts_;
  int reconnection_delay_ms_;

  const double G_ACC = 9.80665;
  const double GYRO_FACTOR = 2000.0 / 32800.0;
  const double ACCEL_FACTOR = 2.0 / 32768.0;
  const bool DEBUG_PRINT;

  void queueThread();

  void publishIMU();
  void publishButton();
  double lowPassFilter(double alpha, double x_new, double& x_old);
  bool attempt_reconnection();

  int control_cycle_msec_;
  std::thread queue_thread_;
  std::map<std::string, rclcpp::Time> buttons_press_time_;
  rclcpp::Time button_press_time_;
  rclcpp::Time last_msg_time_;
  std::map<std::string, double> previous_result_;
  
  sensor_msgs::msg::Imu imu_msg_;
  double button_msg_;
  
  /* subscriber & publisher */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr button_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_msg_pub_;

  std::atomic<bool> stop_thread;
};

}  // namespace robotis_op

#endif /* SUBCON_MODULE_HPP_ */