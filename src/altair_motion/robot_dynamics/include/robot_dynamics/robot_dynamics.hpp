#ifndef ROBOT_DYNAMICS_HPP_
#define ROBOT_DYNAMICS_HPP_

#include <thread>
#include <map>
#include <iostream>
#include <string>
#include <atomic>
#include <cstdlib>
#include <memory>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "robotis_framework_common/sensor_module.h"
#include "robotis_controller_msgs/msg/joint_ctrl_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/msg/status_msg.hpp"
#include "robotis_math/robotis_math.h"

#ifdef SENSOR
#undef SENSOR
#endif

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/spatial/se3.hpp"

namespace pin = pinocchio;
namespace robot_dynamics
{
class RobotDynamics : public robotis_framework::SensorModule,
                      public robotis_framework::Singleton<RobotDynamics>,
                      public rclcpp::Node
{
public:
  RobotDynamics();
  virtual ~RobotDynamics();

  // ROS Framework Functions
  void initialize(const int control_cycle_msec, robotis_framework::Robot* robot) override;
  void process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
               std::map<std::string, robotis_framework::Sensor*> sensors) override;

private:
  pin::Model model_;
  pin::Data data_;
  Eigen::VectorXd q_;
  pin::FrameIndex left_leg_id_;
  pin::FrameIndex right_leg_id_;
  pin::FrameIndex cam_id_;
  pin::FrameIndex body_id_;

  pin::SE3 world_to_left_leg_;
  pin::SE3 world_to_right_leg_;
  pin::SE3 world_to_body_;
  pin::SE3 world_to_camera_;
  double zL_;
  double zR_;
  bool left_stance_;
  pin::SE3 world_to_leg;
  pin::SE3 leg_to_body_;
  pin::SE3 body_to_cam_;

  sensor_msgs::msg::Imu imu_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr zmp_pub_;

  void queueThread();
  void publishStatusMsg(unsigned int type, std::string msg);
  void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg);

  int control_cycle_msec_;
  std::unique_ptr<std::thread> queue_thread_;
  std::atomic<bool> queue_thread_running_{ false };

  int mass;
  Eigen::Vector3d prev_trans_;
  double prev_yaw_;
};

}  // namespace robot_dynamics

#endif  // ROBOT_DYNAMICS_HPP_
