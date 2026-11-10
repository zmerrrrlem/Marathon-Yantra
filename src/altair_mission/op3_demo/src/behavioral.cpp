#include "op3_demo/behavioral.hpp"
#include <numeric>

namespace altair_mission
{
Behavioral::Behavioral() : Node("op3_demo")
{
  queue_thread_ = std::make_unique<std::thread>(&Behavioral::queueThread, this);
  std::cout << "Behavioral Node" << std::endl;
}

Behavioral::~Behavioral()
{
  std::cout << "Deactivate Behavioral" << std::endl;
}

void Behavioral::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  // service client
  is_running_action_ = this->create_client<op3_action_module_msgs::srv::IsRunning>("/robotis/action/is_running");

  // publisher
  motion_index_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robotis/action/page_num", 0);
  set_module_ =
      this->create_publisher<robotis_controller_msgs::msg::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // subscribers
  button_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/altair/button", 10, std::bind(&Behavioral::buttonCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/altair/imu", 10, std::bind(&Behavioral::imuCallback, this, std::placeholders::_1));

  rclcpp::Rate rate(30);
  while (rclcpp::ok())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void Behavioral::buttonCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Handle button input
  RCLCPP_INFO(this->get_logger(), "Button value: %f", msg->data);
}

void Behavioral::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Handle IMU data
}

void Behavioral::setModuleTo(const std::string& module_name)
{
  robotis_controller_msgs::msg::JointCtrlModule control_msg;
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->second);
    control_msg.module_name.push_back(module_name);
  }

  if (control_msg.joint_name.size() == 0)
    return;

  set_module_->publish(control_msg);
}

void Behavioral::setBodyModuleTo(const std::string& body_module, bool with_head_control)
{
  robotis_controller_msgs::msg::JointCtrlModule control_msg;

  std::string head_module = "head_control_module";
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    if (joint_iter->second.find("head") != std::string::npos)
    {
      if (with_head_control == true)
      {
        RCLCPP_WARN(this->get_logger(), "Head is Enabled");
        control_msg.joint_name.push_back(joint_iter->second);
        control_msg.module_name.push_back(head_module);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Head is Disabled");
        continue;
      }
    }
    else
    {
      control_msg.joint_name.push_back(joint_iter->second);
      control_msg.module_name.push_back(body_module);
    }
  }

  if (control_msg.joint_name.size() == 0)
    return;

  set_module_->publish(control_msg);
}

void Behavioral::setWalkingParam(double x_move, double y_move, double rotation_angle)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = x_move;
  cmd_vel.linear.y = y_move;
  cmd_vel.angular.z = rotation_angle;

  if (!CALIBRATION)
    cmd_vel_pub_->publish(cmd_vel);
}

void Behavioral::calcWalkingParam(double x_move, double y_move, double rotation_angle)
{
  float delta_FB = fabs(FB_Step - x_move);
  float delta_RLS = fabs(RL_Step - y_move);
  float delta_RLT = fabs(RL_Turn - rotation_angle);
  float max_ = std::max(delta_RLS, delta_RLT);
  max_ = std::max(delta_FB, max_);
  float score = max_ / UNIT_INCREMENT;

  if (score != 0)
  {
    increment_fb = delta_FB / score;
    increment_rl = delta_RLS / score;
    increment_turn = delta_RLT / score;
  }
  else
    increment_fb = increment_rl = increment_turn = 0;

  // CALCULATION
  // FB MOVE
  if (x_move >= FB_Step)
  {
    if (x_move == 0)
      FB_Step = 0;
    else
      FB_Step += increment_fb * 2;
  }
  else if (x_move < FB_Step)
  {
    if (x_move == 0)
      FB_Step = 0;
    else
      FB_Step -= increment_fb * 2;
  }

  // RL MOVE
  if (y_move >= RL_Step)
  {
    if (y_move == 0)
      RL_Step = 0;
    else
      RL_Step += increment_rl;
  }
  else if (y_move < RL_Step)
  {
    if (y_move == 0)
      RL_Step = 0;
    else
      RL_Step -= increment_rl;
  }

  // RL TURN
  if (rotation_angle >= RL_Turn)
  {
    if (rotation_angle > RL_Turn)
      RL_Turn += increment_turn;
    else if (rotation_angle == 0)
      RL_Turn = 0;
  }
  else if (rotation_angle < RL_Turn)
  {
    if (rotation_angle < RL_Turn)
      RL_Turn -= increment_turn;
    else if (rotation_angle == 0)
      RL_Turn = 0;
  }

  // CHECK VALUE
  // FB MOVE
  if (FB_Step > MAX_FB_STEP)
    FB_Step = MAX_FB_STEP;
  else if (FB_Step < MAX_BW_STEP)
    FB_Step = MAX_BW_STEP;

  // RL MOVE
  if (RL_Step > MAX_RL_STEP)
    RL_Step = MAX_RL_STEP;
  else if (RL_Step < -MAX_RL_STEP)
    RL_Step = -MAX_RL_STEP;

  // RL TURN
  if (RL_Turn > MAX_RL_TURN)
    RL_Turn = MAX_RL_TURN;
  else if (RL_Turn < -MAX_RL_TURN)
    RL_Turn = -MAX_RL_TURN;

  setWalkingParam(FB_Step, RL_Step, RL_Turn);
}

bool Behavioral::GoingToBall(double robot_theta, double ball_x, double ball_y, double ball_theta)
{
  RCLCPP_INFO(this->get_logger(), "GO TO BALL");

  static bool rotate = false;
  static int counterAction = 0;
  static std::vector<int> ball_x_ma, ball_y_ma;

  int size_buff = 15;
  double ball_x_filt, ball_y_filt;

  ball_x_ma.insert(ball_x_ma.end(), ball_x);
  ball_y_ma.insert(ball_y_ma.end(), ball_y);

  double s_angle_yaw;
  if (resetOffset > 0)
  {
    if (angle_yaw > 0)
      s_angle_yaw = angle_yaw - resetOffset;
    else
      s_angle_yaw = angle_yaw + resetOffset;
  }
  else
  {
    if (angle_yaw > 0)
      s_angle_yaw = angle_yaw + resetOffset;
    else
      s_angle_yaw = angle_yaw - resetOffset;
  }

  if (ball_x_ma.size() < size_buff)
  {
    ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / ball_x_ma.size());
    ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / ball_y_ma.size());
  }
  else
  {
    ball_x_filt = double(std::accumulate(ball_x_ma.begin(), ball_x_ma.end(), 0) / size_buff);
    ball_x_ma.erase(ball_x_ma.begin());
    ball_y_filt = double(std::accumulate(ball_y_ma.begin(), ball_y_ma.end(), 0) / size_buff);
    ball_y_ma.erase(ball_y_ma.begin());
  }

  ball_x = ball_x_filt - 9.6;
  ball_y = ball_y_filt;

  double TOTAL_THETA = s_angle_yaw - ball_theta;
  double RB_THETA = atan2((ball_y), fabs(ball_x)) * 180 / M_PI;

  if (ball_x <= FORWARD_MIN_LIMIT && fabs(ball_y) == RL_MIN_LIMIT && fabs(TOTAL_THETA) < KICKING_TOLERANCE)
  {
    RCLCPP_ERROR(this->get_logger(), "READY TO KICK");
    lastKick = true;
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "BALL X [%f], BALL Y[%f], RB_THETA[%f], TOTAL THETA[%f]", ball_x, ball_y, RB_THETA,
              TOTAL_THETA);

  switch (followState)
  {
    case Following_Forward:
      RCLCPP_INFO(this->get_logger(), "FORWARD");
      counterAction++;
      RCLCPP_INFO(this->get_logger(), "Cnt Action: %d", counterAction);

      if (ball_x < FORWARD_MIN_LIMIT && ((ball_y <= 10 && ball_y >= 1) || (ball_y <= -1 && ball_y >= -10)) &&
          fabs(TOTAL_THETA) < KICKING_TOLERANCE)
      {
        followState = Following_Stop;
        break;
      }

      if (ball_x > 40)
      {
        counterAction = 0;
        calcWalkingParam(0.3, -0.05, 0.37);

        if (fabs(RB_THETA) > 1.5 * BALL_TOLERANCE)
        {
          followState = Following_Rotate;
          break;
        }
      }
      else if (counterAction > 20)
      {
        if (fabs(RB_THETA) > BALL_TOLERANCE)
          followState = Following_Rotate;
        else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
          followState = Following_Revo;
        else if (ball_x < FORWARD_MIN_REVO)
          followState = Following_ForwardMin;
        else
          calcWalkingParam(0.3, -0.05, 0.37);
      }
      break;

    case Following_Rotate:
      if (ball_x < FORWARD_MIN_LIMIT && ((ball_y <= 20 && ball_y >= 1) || (ball_y <= -1 && ball_y >= -20)) &&
          fabs(TOTAL_THETA) < KICKING_TOLERANCE)
      {
        followState = Following_Stop;
        break;
      }

      if (rotate)
      {
        RCLCPP_INFO(this->get_logger(), "ROTATE_ANJAY");

        if (ball_x > 70 && fabs(RB_THETA) <= BALL_TOLERANCE)
          rotate = false;

        setWalkingParam(((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.0 : -0.002,
                        ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.1 : 0.2,
                        ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.00 : -1.0);
      }
      else if (ball_x > 160)
      {
        RCLCPP_INFO(this->get_logger(), "ROTATE_FORWARD_JAUUHHHHH");

        if (fabs(ball_y) > 125)
          rotate = true;

        setWalkingParam(0.3, ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 0.15 : -0.15,
                        ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -1 : 1);
      }
      else
      {
        if (fabs(ball_y) > 30)
          followState = Following_Shift;
        else if (fabs(RB_THETA) > BALL_TOLERANCE)
          rotate = true;
        else if (ball_x < 60 && ball_x < 60)
          followState = Following_ForwardMin;
        else if (ball_x <= 14 && fabs(ball_y) >= 4)
          followState = Following_Shift;
        else if (fabs(RB_THETA) < REVO_TOLERANCE && ball_x < FORWARD_MIN_REVO)
          followState = Following_Revo;

        setWalkingParam(0.25, ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? -0.2 : 0.2,
                        ((RB_THETA > 0 && RB_THETA < 90) || RB_THETA <= -90) ? 1.0 : -1);
      }

      if (fabs(RB_THETA) < BALL_TOLERANCE)
        followState = Following_Forward;
      break;

    case Following_Revo:
      RCLCPP_INFO(this->get_logger(), "REVO");

      if (TOTAL_THETA > 0)
      {
        RCLCPP_INFO(this->get_logger(), "============ CCW ============");
        setWalkingParam(0.15, -0.3, 0.5);
      }
      else if (TOTAL_THETA < 0)
      {
        RCLCPP_INFO(this->get_logger(), "============ CW ============");
        setWalkingParam(0.1, 0.2, -0.5);
      }

      if (ball_x > FORWARD_MIN_REVO)
      {
        if (fabs(RB_THETA) > BALL_TOLERANCE)
          followState = Following_Rotate;
        else
          followState = Following_ForwardMin;
      }
      else
      {
        if (fabs(TOTAL_THETA) < ANGLE_TOLERANCE)
        {
          if (fabs(ball_y) > RL_MIN_LIMIT)
            followState = Following_Shift;
          else
            followState = Following_ForwardMin;
        }
      }

      if (ball_x < FORWARD_MIN_LIMIT && ((ball_y <= 10 && ball_y >= 1) || (ball_y <= -1 && ball_y >= -10)) &&
          fabs(TOTAL_THETA) < KICKING_TOLERANCE)
      {
        followState = Following_Stop;
        break;
      }
      break;

    case Following_Shift:
      RCLCPP_INFO(this->get_logger(), "SHIFT");

      if (ball_x < FORWARD_MIN_LIMIT && ((ball_y <= 10 && ball_y >= 1) || (ball_y <= -1 && ball_y >= -10)) &&
          fabs(TOTAL_THETA) < KICKING_TOLERANCE)
      {
        followState = Following_Stop;
        break;
      }

      if (ball_y < -1)
      {
        RCLCPP_ERROR(this->get_logger(), "GESER KANAN");
        setWalkingParam(-0.1, -0.23, -0.3);
      }
      else if (ball_y > 1)
      {
        RCLCPP_ERROR(this->get_logger(), "GESER KIRI");
        setWalkingParam(-0.06, 0.2, 0.450);
      }

      if (fabs(ball_y) <= RL_MIN_LIMIT && ball_x > FORWARD_MIN_REVO)
      {
        if (fabs(RB_THETA) > BALL_TOLERANCE)
          followState = Following_Rotate;
        else
          followState = Following_ForwardMin;
      }
      else if (ball_x > 0 && fabs(ball_y) < RL_MIN_LIMIT)
      {
        if (ball_x > FORWARD_MIN_LIMIT)
          followState = Following_ForwardMin;
        else if (fabs(TOTAL_THETA) > ANGLE_TOLERANCE)
          followState = Following_Revo;
      }
      break;

    case Following_ForwardMin:
      RCLCPP_INFO(this->get_logger(), "FORWARD MIN");
      setWalkingParam(0.15, 0.04, -0.24);

      if (ball_x <= FORWARD_MIN_LIMIT && ((ball_y <= 13 && ball_y >= 1) || (ball_y <= -1 && ball_y >= -13)) &&
          fabs(TOTAL_THETA) < KICKING_TOLERANCE)
      {
        followState = Following_Stop;
        break;
      }

      if (ball_x > FORWARD_MIN_REVO)
      {
        if (fabs(RB_THETA) > BALL_TOLERANCE)
          followState = Following_Rotate;
        else
          followState = Following_Forward;
      }
      else if (fabs(ball_y) >= (RL_MIN_LIMIT + 4) && ball_x <= (FORWARD_MIN_LIMIT + 1))
        followState = Following_Shift;
      break;

    case Following_Backward:
      RCLCPP_INFO(this->get_logger(), "BACKWARD");
      setWalkingParam(MAX_BW_STEP, 0, 0.3);

      if (ball_x > FORWARD_MIN_REVO)
      {
        if (fabs(RB_THETA) > BALL_TOLERANCE)
          followState = Following_Rotate;
        else
          followState = Following_ForwardMin;
      }
      else if (fabs(ball_x) <= FORWARD_MIN_REVO && fabs(ball_y) >= 5)
        followState = Following_Shift;
      else if (fabs(ball_x <= FORWARD_MIN_REVO))
        followState = Following_Revo;
      break;

    case Following_Stop:
      RCLCPP_INFO(this->get_logger(), "READY TO KICK");
      lastKick = true;
      return true;

    default:
      RCLCPP_INFO(this->get_logger(), "GAMASUK STATE WOYYYY");
      followState = Following_ForwardMin;
      break;
  }

  return false;
}

}  // namespace altair_mission