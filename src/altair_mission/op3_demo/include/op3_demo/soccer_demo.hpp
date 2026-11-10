#ifndef OP3_DEMO__SOCCER_DEMO_HPP_
#define OP3_DEMO__SOCCER_DEMO_HPP_

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "op3_action_module_msgs/srv/is_running.hpp"

namespace altair_mission
{

class SoccerDemo : public rclcpp::Node
{
public:
  SoccerDemo();
  ~SoccerDemo();

  void setModuleEnable();
  void setModuleDisable();

private:
  // Status enums
  enum SoccerStatus
  {
    Soccer_Stop = 0,
    Soccer_Start = 1,
    Soccer_Running = 2,
    Soccer_Waiting = 3
  };

  enum FollowingState
  {
    Following_Forward = 0,
    Following_Shift = 1,
    Following_Rotate = 2,
    Following_Stop = 3
  };

  enum StandState
  {
    Stand = 0,
    Nothing = 1,
    FallenForward = 2,
    FallenBackward = 3
  };

  // Constants
  static constexpr int SPIN_RATE = 30;
  static constexpr bool DEBUG_PRINT = true;
  
  // Ball detection thresholds
  static constexpr double FORWARD_MIN_LIMIT = 15.0;   // cm
  static constexpr double RL_MIN_LIMIT = 8.0;         // cm
  static constexpr double BALL_TOLERANCE = 10.0;      // cm
  static constexpr double ANGLE_TOLERANCE = 5.0;      // degrees
  
  // Walking parameters limits
  static constexpr double MAX_FB_STEP = 0.04;         // m/s
  static constexpr double MAX_BW_STEP = -0.02;        // m/s
  static constexpr double MAX_RL_STEP = 0.03;         // m/s
  static constexpr double MAX_RL_TURN = 0.5;          // rad/s
  static constexpr double UNIT_INCREMENT = 0.005;

  // Thread functions
  void queueThread();
  void processThread();
  
  // Callbacks
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void ballPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void statusCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg);
  
  // Soccer state machine
  void startSoccerMode();
  void stopSoccerMode();
  bool soccerStateMachine();
  void ballFollowingBehavior();
  
  // Module control
  void setModuleTo(const std::string& module_name);
  void setBodyModuleTo(const std::string& body_module, bool with_head_control = true);
  bool isActionRunning();
  void playMotion(int motion_index);
  
  // Walking control
  void setWalkingParam(double x_move, double y_move, double rotation_angle);
  void calcWalkingParam(double x_move, double y_move, double rotation_angle);
  void stopWalking();
  
  // Ball following logic
  bool goingToBall(double ball_x, double ball_y, double robot_theta);
  
  // ROS2 interfaces
  rclcpp::Publisher<robotis_controller_msgs::msg::JointCtrlModule>::SharedPtr set_module_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motion_index_pub_;
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_position_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<robotis_controller_msgs::msg::StatusMsg>::SharedPtr status_sub_;
  
  rclcpp::Client<op3_action_module_msgs::srv::IsRunning>::SharedPtr is_running_client_;
  
  // Threads
  std::unique_ptr<std::thread> queue_thread_;
  std::unique_ptr<std::thread> process_thread_;
  
  // State variables
  bool enable_;
  int soccer_state_;
  int following_state_;
  int stand_state_;
  
  // Ball tracking data
  double ball_x_;
  double ball_y_;
  double ball_theta_;
  bool ball_found_;
  rclcpp::Time last_ball_time_;
  
  // IMU data
  double angle_yaw_;
  double angle_pitch_;
  double angle_roll_;
  
  // Walking parameters
  double FB_Step_;
  double RL_Step_;
  double RL_Turn_;
  double increment_fb_;
  double increment_rl_;
  double increment_turn_;
  
  // Joint table
  std::map<int, std::string> id_joint_table_;
  
  // Manager status
  bool manager_ready_;
};

}  // namespace altair_mission

#endif  // OP3_DEMO__SOCCER_DEMO_HPP_
