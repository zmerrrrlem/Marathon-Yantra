#ifndef BEHAVIORAL_HPP_
#define BEHAVIORAL_HPP_

#include <iostream>
#include <thread>
#include <map>
#include <iostream>
#include <string>
#include <atomic>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "op3_action_module_msgs/srv/is_running.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robotis_math/robotis_math.h"

namespace altair_mission
{
class Behavioral : public rclcpp::Node
{
public:
  Behavioral();
  ~Behavioral();

  void setModuleTo(const std::string& module_name);
  void setBodyModuleTo(const std::string& body_module, bool with_head_control = true);
  
  // Following functions
  bool GoingToBall(double robot_theta, double ball_x, double ball_y, double ball_theta);
  void setWalkingParam(double x_move, double y_move, double rotation_angle);
  void calcWalkingParam(double x_move, double y_move, double rotation_angle);

private:
  void queueThread();
  std::unique_ptr<std::thread> queue_thread_;

  // callbacks
  void buttonCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // service
  rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedPtr is_running_action_;
  
  // publisher
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr set_module_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // subscriber
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr button_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Member variables for joint control
  std::map<int, std::string> id_joint_table_;
  
  // Following state machine
  enum Following_State
  {
    Following_Forward = 0,
    Following_Shift = 1,
    Following_Rotate = 2,
    Following_Revo = 3,
    Following_ForwardMin = 4,
    Following_Backward = 5,
    Following_OnTheSpot = 6,
    Following_Trantition = 7,
    Following_Stop = 8,
  };
  
  int followState;
  bool lastKick;
  double angle_yaw;
  double resetOffset;
  
  // Walking parameters
  double FB_Step, RL_Step, RL_Turn;
  double increment_fb, increment_rl, increment_turn;
  
  // Constants (these should be initialized in constructor or loaded from config)
  double FORWARD_MIN_LIMIT = 10.0;
  double RL_MIN_LIMIT = 5.0;
  double KICKING_TOLERANCE = 10.0;
  double BALL_TOLERANCE = 8.0;
  double REVO_TOLERANCE = 5.0;
  double FORWARD_MIN_REVO = 20.0;
  double ANGLE_TOLERANCE = 5.0;
  double MAX_FB_STEP = 0.4;
  double MAX_BW_STEP = -0.15;
  double MAX_RL_STEP = 0.3;
  double MAX_RL_TURN = 1.0;
  double MIN_FB_STEP = 0.15;
  double UNIT_INCREMENT = 0.05;
  bool CALIBRATION = false;
};

}  // namespace altair_mission

#endif