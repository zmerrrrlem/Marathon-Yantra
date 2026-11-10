#include "robot_dynamics/robot_dynamics.hpp"

using std::placeholders::_1;
namespace robot_dynamics
{
RobotDynamics::RobotDynamics() : Node("robot_dynamics_module"), control_cycle_msec_(8)
{
  module_name_ = "robot_dynamics_module";
}

RobotDynamics::~RobotDynamics()
{
  queue_thread_running_.store(false);
  if (queue_thread_ && queue_thread_->joinable())
    queue_thread_->join();
  queue_thread_.reset();
}

void RobotDynamics::initialize(const int control_cycle_msec_, robotis_framework::Robot* robot)
{
  const auto filename = "altair.urdf";
  auto urdf = ament_index_cpp::get_package_share_directory("altair_description") + "/urdf/" + filename;
  pin::urdf::buildModel(urdf, pin::JointModelFreeFlyer(), model_);
  data_ = pin::Data(model_);
  q_ = Eigen::VectorXd::Zero(model_.nq);
  pin::forwardKinematics(model_, data_, q_);
  pin::updateFramePlacements(model_, data_);
  left_leg_id_ = model_.getFrameId("l_ank_roll_link");
  right_leg_id_ = model_.getFrameId("r_ank_roll_link");
  body_id_ = model_.getFrameId("body_link");
  cam_id_ = model_.getFrameId("cam_link");

  // World to link frames
  world_to_left_leg_ = data_.oMf[left_leg_id_];
  world_to_right_leg_ = data_.oMf[right_leg_id_];
  world_to_body_ = data_.oMf[body_id_];
  world_to_camera_ = data_.oMf[cam_id_];

  queue_thread_running_.store(true);
  queue_thread_ = std::make_unique<std::thread>(&RobotDynamics::queueThread, this);
  RCLCPP_INFO(this->get_logger(), "RobotDynamics: initialize");
}

void RobotDynamics::queueThread()
{
  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(this->get_node_base_interface());

  // subscriber and publisher
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("robotis_op3/imu", 1,
                                                              std::bind(&RobotDynamics::imuCallback, this, _1));

  zmp_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_dynamics/zmp_estimated", 10);

  rclcpp::Rate rate(1000.0 / control_cycle_msec_);
  while (rclcpp::ok() && queue_thread_running_.load())
  {
    executor.spin_some();
    rate.sleep();
  }
}

void RobotDynamics::process(std::map<std::string, robotis_framework::Dynamixel*> dxls,
                            std::map<std::string, robotis_framework::Sensor*> sensors)
{
  // std::cout << "====================\n";
  // dxl position is in radian
  for (auto [joint_name, state] : dxls)
  {
    pin::JointIndex joint_index = model_.getJointId(joint_name);
    int idx_q = model_.joints[joint_index].idx_q();
    q_[idx_q] = state->dxl_state_->goal_position_;
  }

  Eigen::Quaterniond rotation_of_world_to_body(imu_.orientation.w, imu_.orientation.x, imu_.orientation.y,
                                               imu_.orientation.z);
  q_.segment<3>(0).setZero();
  q_[3] = rotation_of_world_to_body.x();
  q_[4] = rotation_of_world_to_body.y();
  q_[5] = rotation_of_world_to_body.z();
  q_[6] = rotation_of_world_to_body.w();

  pin::forwardKinematics(model_, data_, q_);
  pin::updateFramePlacements(model_, data_);

  world_to_left_leg_ = data_.oMf[left_leg_id_];
  world_to_right_leg_ = data_.oMf[right_leg_id_];
  world_to_body_ = data_.oMf[body_id_];
  world_to_camera_ = data_.oMf[cam_id_];

  zL_ = world_to_left_leg_.translation().z();
  zR_ = world_to_right_leg_.translation().z();
  // check stance leg
  left_stance_ = (zL_ < zR_);

  // Base pose relative to stance leg
  world_to_leg = left_stance_ ? world_to_left_leg_ : world_to_right_leg_;

  leg_to_body_ = world_to_leg.inverse() * world_to_body_;      // stance leg -> body
  body_to_cam_ = world_to_body_.inverse() * world_to_camera_;  // body -> camera

  geometry_msgs::msg::Pose base_3d;
  base_3d.position.x = leg_to_body_.translation().x();
  base_3d.position.y = leg_to_body_.translation().y();
  base_3d.position.z = leg_to_body_.translation().z();

  Eigen::Quaterniond leg_to_body_quaternion(leg_to_body_.rotation());
  base_3d.orientation.x = leg_to_body_quaternion.x();
  base_3d.orientation.y = leg_to_body_quaternion.y();
  base_3d.orientation.z = leg_to_body_quaternion.z();
  base_3d.orientation.w = leg_to_body_quaternion.w();

  // base_pose_.publish(base_3d);  // ROS2 publisher, analogous to old base_pose_.publish(base_3d);

  // 6D Camera pose (world frame)
  geometry_msgs::msg::PoseStamped cam_pose;
  cam_pose.header.stamp = this->now();
  cam_pose.header.frame_id = "camera";

  const Eigen::Vector3d t_WC = world_to_camera_.translation();
  const Eigen::Matrix3d R_WC = world_to_camera_.rotation();
  Eigen::Quaterniond q_WC(R_WC);

  cam_pose.pose.position.x = t_WC.x();
  cam_pose.pose.position.y = t_WC.y();
  cam_pose.pose.position.z = t_WC.z();
  cam_pose.pose.orientation.x = q_WC.x();
  cam_pose.pose.orientation.y = q_WC.y();
  cam_pose.pose.orientation.z = q_WC.z();
  cam_pose.pose.orientation.w = q_WC.w();

  // camera_pub_.publish(cam_pose);

  // 2D odometry
  Eigen::Vector3d rpy = robotis_framework::convertQuaternionToRPY(rotation_of_world_to_body);  // roll, pitch, yaw
  double yaw = rpy.z();

  Eigen::Vector3d dif_pose = leg_to_body_.translation() - prev_trans_;
  double dif_yaw = yaw - prev_yaw_;

  geometry_msgs::msg::Pose2D base_2d;
  base_2d.x = dif_pose.x();
  base_2d.y = dif_pose.y();
  base_2d.theta = dif_yaw;
  // std::cout << "x: " << (double)base_2d.x << std::endl;
  // std::cout << "x: " << (double)base_2d.y << std::endl;

  // base2d_.publish(base_2d);

  prev_trans_ = leg_to_body_.translation();
  prev_yaw_ = yaw;

  // ZMP ESTIMATION
  const double g = 9.80665;

  pin::centerOfMass(model_, data_, q_);
  Eigen::Vector3d com_pos = data_.com[0];  // world frame CoM position

  static Eigen::Vector3d prev_com = com_pos;
  static Eigen::Vector3d prev_dcom = Eigen::Vector3d::Zero();
  double dt = control_cycle_msec_ / 1000.0;
  Eigen::Vector3d dcom = (com_pos - prev_com) / dt;
  Eigen::Vector3d ddcom = (dcom - prev_dcom) / dt;

  prev_com = com_pos;
  prev_dcom = dcom;

  // zmp estimation
  // zmp_x = com_x - (z_com / g) * ddx_com
  // zmp_y = com_y - (z_com / g) * ddy_com
  double zmp_x = com_pos.x() - (com_pos.z() / g) * ddcom.x();
  double zmp_y = com_pos.y() - (com_pos.z() / g) * ddcom.y();

  Eigen::Vector3d zmp_world(zmp_x, zmp_y, 0.0);
  Eigen::Vector3d zmp_leg = world_to_leg.inverse().act(zmp_world);

  geometry_msgs::msg::PointStamped zmp_msg;
  zmp_msg.header.stamp = this->now();
  zmp_msg.header.frame_id = left_stance_ ? "l_ank_roll_link" : "r_ank_roll_link";
  zmp_msg.point.x = zmp_leg.x();
  zmp_msg.point.y = zmp_leg.y();
  zmp_msg.point.z = 0.0;
  // std::cout << "zmp estimate x: " << zmp_leg.x() << " y: " << zmp_leg.y() << std::endl;

  zmp_pub_->publish(zmp_msg);
}

void RobotDynamics::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_ = *msg;
}

}  // namespace robot_dynamics
