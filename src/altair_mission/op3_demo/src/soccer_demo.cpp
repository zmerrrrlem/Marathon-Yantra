#include "op3_demo/soccer_demo.hpp"
#include <cmath>
#include <algorithm>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace altair_mission
{

SoccerDemo::SoccerDemo()
  : Node("soccer_demo"),
    enable_(false),
    soccer_state_(Soccer_Stop),
    following_state_(Following_Stop),
    stand_state_(Nothing),
    ball_x_(0.0),
    ball_y_(0.0),
    ball_theta_(0.0),
    ball_found_(false),
    angle_yaw_(0.0),
    angle_pitch_(0.0),
    angle_roll_(0.0),
    FB_Step_(0.0),
    RL_Step_(0.0),
    RL_Turn_(0.0),
    increment_fb_(0.0),
    increment_rl_(0.0),
    increment_turn_(0.0),
    manager_ready_(false)
{
  RCLCPP_INFO(this->get_logger(), "Soccer Demo Node Initialized");
  
  // Initialize joint table (add your robot's joints here)
  id_joint_table_[1] = "r_sho_pitch";
  id_joint_table_[2] = "l_sho_pitch";
  id_joint_table_[3] = "r_sho_roll";
  id_joint_table_[4] = "l_sho_roll";
  id_joint_table_[5] = "r_el";
  id_joint_table_[6] = "l_el";
  id_joint_table_[7] = "r_hip_yaw";
  id_joint_table_[8] = "l_hip_yaw";
  id_joint_table_[9] = "r_hip_roll";
  id_joint_table_[10] = "l_hip_roll";
  id_joint_table_[11] = "r_hip_pitch";
  id_joint_table_[12] = "l_hip_pitch";
  id_joint_table_[13] = "r_knee";
  id_joint_table_[14] = "l_knee";
  id_joint_table_[15] = "r_ank_pitch";
  id_joint_table_[16] = "l_ank_pitch";
  id_joint_table_[17] = "r_ank_roll";
  id_joint_table_[18] = "l_ank_roll";
  id_joint_table_[19] = "head_pan";
  id_joint_table_[20] = "head_tilt";
  
  // Start threads
  queue_thread_ = std::make_unique<std::thread>(&SoccerDemo::queueThread, this);
  process_thread_ = std::make_unique<std::thread>(&SoccerDemo::processThread, this);
}

SoccerDemo::~SoccerDemo()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Soccer Demo");
  enable_ = false;
  
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();
  if (process_thread_ && process_thread_->joinable())
    process_thread_->join();
}

void SoccerDemo::queueThread()
{
  // Publishers
  set_module_pub_ = this->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>(
      "/robotis/set_joint_ctrl_modules", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
  motion_index_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/robotis/action/page_num", 10);
  
  // Subscribers
  command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/soccer_demo/command", 10,
      std::bind(&SoccerDemo::commandCallback, this, std::placeholders::_1));
  
  ball_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/ball_tracker/position", 10,
      std::bind(&SoccerDemo::ballPositionCallback, this, std::placeholders::_1));
  
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/altair/imu", 10,
      std::bind(&SoccerDemo::imuCallback, this, std::placeholders::_1));
  
  status_sub_ = this->create_subscription<robotis_controller_msgs::msg::StatusMsg>(
      "/robotis/status", 10,
      std::bind(&SoccerDemo::statusCallback, this, std::placeholders::_1));
  
  // Service clients
  is_running_client_ = this->create_client<op3_action_module_msgs::srv::IsRunning>(
      "/robotis/action/is_running");
  
  RCLCPP_INFO(this->get_logger(), "Queue thread started");
}

void SoccerDemo::processThread()
{
  rclcpp::Rate loop_rate(SPIN_RATE);
  
  RCLCPP_INFO(this->get_logger(), "Process thread started");
  
  while (rclcpp::ok())
  {
    if (enable_)
    {
      // Run soccer state machine
      if (soccerStateMachine())
      {
        // Execute ball following behavior
        ballFollowingBehavior();
      }
    }
    
    loop_rate.sleep();
  }
}

void SoccerDemo::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!enable_)
    return;
  
  if (msg->data == "start")
  {
    RCLCPP_INFO(this->get_logger(), "Starting soccer mode");
    startSoccerMode();
  }
  else if (msg->data == "stop")
  {
    RCLCPP_INFO(this->get_logger(), "Stopping soccer mode");
    stopSoccerMode();
  }
}

void SoccerDemo::ballPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Update ball position from vision system
  // Assuming msg contains: x (distance), y (lateral offset), z (angle/bearing)
  ball_x_ = msg->x;
  ball_y_ = msg->y;
  ball_theta_ = msg->z;
  
  // Check if ball is detected (assuming 999 means not found)
  if (ball_x_ != 999.0 && ball_y_ != 999.0)
  {
    ball_found_ = true;
    last_ball_time_ = this->now();
  }
  else
  {
    ball_found_ = false;
  }
}

void SoccerDemo::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Extract orientation from quaternion
  tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);
  
  tf2::Matrix3x3 m(q);
  m.getRPY(angle_roll_, angle_pitch_, angle_yaw_);
  
  // Convert to degrees
  angle_yaw_ = angle_yaw_ * 180.0 / M_PI;
  angle_pitch_ = angle_pitch_ * 180.0 / M_PI;
  angle_roll_ = angle_roll_ * 180.0 / M_PI;
}

void SoccerDemo::statusCallback(const robotis_controller_msgs::msg::StatusMsg::SharedPtr msg)
{
  if (msg->status_msg == "Finish Init Pose")
  {
    manager_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "Manager is ready");
  }
}

void SoccerDemo::setModuleEnable()
{
  enable_ = true;
  startSoccerMode();
}

void SoccerDemo::setModuleDisable()
{
  enable_ = false;
  stopWalking();
  setBodyModuleTo("none", false);
}

void SoccerDemo::startSoccerMode()
{
  RCLCPP_INFO(this->get_logger(), "Starting Soccer Mode");
  enable_ = true;
  soccer_state_ = Soccer_Start;
}

void SoccerDemo::stopSoccerMode()
{
  RCLCPP_INFO(this->get_logger(), "Stopping Soccer Mode");
  enable_ = false;
  soccer_state_ = Soccer_Stop;
  stopWalking();
}

bool SoccerDemo::soccerStateMachine()
{
  switch (soccer_state_)
  {
    case Soccer_Stop:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Soccer: STOP");
      stopWalking();
      return false;
    
    case Soccer_Start:
      RCLCPP_INFO(this->get_logger(), "Soccer: START");
      // Initialize walking module
      setBodyModuleTo("quintic_walk", true);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      soccer_state_ = Soccer_Running;
      return true;
    
    case Soccer_Running:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Soccer: RUNNING");
      return true;
    
    case Soccer_Waiting:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Soccer: WAITING");
      stopWalking();
      return false;
    
    default:
      return false;
  }
}

void SoccerDemo::ballFollowingBehavior()
{
  if (!ball_found_)
  {
    // Ball not found - rotate in place to search
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ball not found - searching...");
    setWalkingParam(0.0, 0.0, 0.2);
    return;
  }
  
  // Ball found - go towards it
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                       "Ball found at x=%.2f, y=%.2f", ball_x_, ball_y_);
  
  goingToBall(ball_x_, ball_y_, angle_yaw_);
}

bool SoccerDemo::goingToBall(double ball_x, double ball_y, double robot_theta)
{
  double FB_Step = 0.0;
  double RL_Step = 0.0;
  double RL_Turn = 0.0;
  
  // Calculate distance and angle to ball
  double ball_distance = std::sqrt(ball_x * ball_x + ball_y * ball_y);
  double ball_angle = std::atan2(ball_y, ball_x) * 180.0 / M_PI;
  
  // State machine for ball following
  if (ball_distance > FORWARD_MIN_LIMIT)
  {
    // Ball is far - move forward
    following_state_ = Following_Forward;
    
    if (std::abs(ball_angle) > ANGLE_TOLERANCE)
    {
      // Need to rotate while moving forward
      FB_Step = MAX_FB_STEP * 0.7;
      RL_Turn = (ball_angle > 0) ? MAX_RL_TURN * 0.5 : -MAX_RL_TURN * 0.5;
    }
    else
    {
      // Move straight forward
      FB_Step = MAX_FB_STEP;
      RL_Turn = 0.0;
    }
  }
  else if (std::abs(ball_y) > RL_MIN_LIMIT)
  {
    // Ball is close but needs lateral adjustment
    following_state_ = Following_Shift;
    FB_Step = MAX_FB_STEP * 0.3;
    RL_Step = (ball_y > 0) ? MAX_RL_STEP : -MAX_RL_STEP;
  }
  else if (std::abs(ball_angle) > ANGLE_TOLERANCE)
  {
    // Ball is in position but need to rotate
    following_state_ = Following_Rotate;
    FB_Step = 0.0;
    RL_Turn = (ball_angle > 0) ? MAX_RL_TURN * 0.3 : -MAX_RL_TURN * 0.3;
  }
  else
  {
    // Ball is in kicking position
    following_state_ = Following_Stop;
    RCLCPP_INFO(this->get_logger(), "Ball in kicking position!");
    stopWalking();
    return true;
  }
  
  // Apply walking parameters with smooth transitions
  calcWalkingParam(FB_Step, RL_Step, RL_Turn);
  setWalkingParam(FB_Step_, RL_Step_, RL_Turn_);
  
  return false;
}

void SoccerDemo::setModuleTo(const std::string& module_name)
{
  robotis_controller_msgs::msg::JointCtrlModule control_msg;
  
  for (const auto& joint_pair : id_joint_table_)
  {
    control_msg.joint_name.push_back(joint_pair.second);
    control_msg.module_name.push_back(module_name);
  }
  
  if (!control_msg.joint_name.empty())
  {
    set_module_pub_->publish(control_msg);
  }
}

void SoccerDemo::setBodyModuleTo(const std::string& body_module, bool with_head_control)
{
  robotis_controller_msgs::msg::JointCtrlModule control_msg;
  std::string head_module = "head_control_module";
  
  for (const auto& joint_pair : id_joint_table_)
  {
    if (joint_pair.second.find("head") != std::string::npos)
    {
      if (with_head_control)
      {
        control_msg.joint_name.push_back(joint_pair.second);
        control_msg.module_name.push_back(head_module);
      }
    }
    else
    {
      control_msg.joint_name.push_back(joint_pair.second);
      control_msg.module_name.push_back(body_module);
    }
  }
  
  if (!control_msg.joint_name.empty())
  {
    set_module_pub_->publish(control_msg);
  }
}

bool SoccerDemo::isActionRunning()
{
  if (!is_running_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_WARN(this->get_logger(), "Action service not available");
    return false;
  }
  
  auto request = std::make_shared<op3_action_module_msgs::srv::IsRunning::Request>();
  auto future = is_running_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    return future.get()->is_running;
  }
  
  return false;
}

void SoccerDemo::playMotion(int motion_index)
{
  std_msgs::msg::Int32 motion_msg;
  motion_msg.data = motion_index;
  motion_index_pub_->publish(motion_msg);
}

void SoccerDemo::setWalkingParam(double x_move, double y_move, double rotation_angle)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = x_move;
  cmd_vel.linear.y = y_move;
  cmd_vel.angular.z = rotation_angle;
  
  cmd_vel_pub_->publish(cmd_vel);
}

void SoccerDemo::calcWalkingParam(double x_move, double y_move, double rotation_angle)
{
  // Calculate deltas
  double delta_FB = std::abs(FB_Step_ - x_move);
  double delta_RLS = std::abs(RL_Step_ - y_move);
  double delta_RLT = std::abs(RL_Turn_ - rotation_angle);
  
  double max_delta = std::max({delta_FB, delta_RLS, delta_RLT});
  double score = max_delta / UNIT_INCREMENT;
  
  if (score > 0)
  {
    increment_fb_ = delta_FB / score;
    increment_rl_ = delta_RLS / score;
    increment_turn_ = delta_RLT / score;
  }
  else
  {
    increment_fb_ = increment_rl_ = increment_turn_ = 0.0;
  }
  
  // Update FB Step
  if (x_move > FB_Step_)
    FB_Step_ += increment_fb_ * 2;
  else if (x_move < FB_Step_)
    FB_Step_ -= increment_fb_ * 2;
  else
    FB_Step_ = x_move;
  
  // Update RL Step
  if (y_move > RL_Step_)
    RL_Step_ += increment_rl_;
  else if (y_move < RL_Step_)
    RL_Step_ -= increment_rl_;
  else
    RL_Step_ = y_move;
  
  // Update RL Turn
  if (rotation_angle > RL_Turn_)
    RL_Turn_ += increment_turn_;
  else if (rotation_angle < RL_Turn_)
    RL_Turn_ -= increment_turn_;
  else
    RL_Turn_ = rotation_angle;
  
  // Clamp values
  FB_Step_ = std::clamp(FB_Step_, MAX_BW_STEP, MAX_FB_STEP);
  RL_Step_ = std::clamp(RL_Step_, -MAX_RL_STEP, MAX_RL_STEP);
  RL_Turn_ = std::clamp(RL_Turn_, -MAX_RL_TURN, MAX_RL_TURN);
}

void SoccerDemo::stopWalking()
{
  FB_Step_ = 0.0;
  RL_Step_ = 0.0;
  RL_Turn_ = 0.0;
  setWalkingParam(0.0, 0.0, 0.0);
}

}  // namespace altair_mission
