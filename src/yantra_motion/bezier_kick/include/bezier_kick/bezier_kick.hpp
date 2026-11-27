#ifndef BEZIER_KICK__BEZIER_KICK_HPP_
#define BEZIER_KICK__BEZIER_KICK_HPP_

#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <chrono>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <fstream>
#include <tuple>
#include <mutex>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rclcpp_lifecycle/state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <chrono>
#include <control_toolbox/pid_ros.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <moveit_msgs/msg/robot_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "biped_interfaces/msg/phase.hpp"
#include "yantra_msgs/msg/foot_pressure.hpp"
#include "yantra_msgs/msg/joint_command.hpp"
#include "yantra_msgs/msg/robot_control_state.hpp"
#include "bezier_kick/kick_ik.hpp"
#include "bezier_kick/kick_utils.hpp"
#include "yantra_splines/abstract_ik.hpp"

#include "bezier_kick_parameters.hpp"

#define PI 3.14159265

namespace bezier_kick
{

  class BezierKick : public controller_interface::ControllerInterface

  {
  public:


    //Parameter Kicking
    bool initPose;
    Eigen::Vector3d L_Shift_XYZ, R_Shift_XYZ, Ankle_Shift_RPY;
    Eigen::Vector3d L_Lift_XYZ, R_Lift_XYZ, L_Lift_RPY, R_Lift_RPY;
    Eigen::Vector3d L_Swing_XYZ, R_Swing_XYZ, L_Swing_RPY, R_Swing_RPY;
    Eigen::Vector3d L_Retract_XYZ, R_Retract_XYZ, L_Retract_RPY, R_Retract_RPY;
    //Parameter Time
    double m_state_time[6];

    //Raw Kick Parameter
    double iTorso_X, iTorso_Y, iTorso_Z; //Init Torso
    double L_Shift_X, L_Shift_Y, L_Shift_Z; //Shift Torso
    double R_Shift_X, R_Shift_Y, R_Shift_Z; //Shift Torso

    double L_Lift_X, L_Lift_Y, L_Lift_Z; //Lift Legs
    double R_Lift_X, R_Lift_Y, R_Lift_Z; //Lift Legs

    double L_Swing_X, L_Swing_Y, L_Swing_Z; //Swing Legs
    double R_Swing_X, R_Swing_Y, R_Swing_Z; //Swing Legs

    double L_Retract_X, L_Retract_Y, L_Retract_Z; //Swing Legs
    double R_Retract_X, R_Retract_Y, R_Retract_Z; //Swing Legs

    double iTorso_Pitch;
    double Shift_Roll;
    double Lift_Roll, Lift_Pitch; //Torso
    double Swing_Roll, Swing_Pitch; //Torso
    double Retract_Roll, Retract_Pitch; //Torso
    

    Eigen::Vector3d BezierKick::bezierCurve2(double phase, Eigen::Vector3d start, Eigen::Vector3d end);
    Eigen::Vector3d BezierKick::bezierCurve3(double phase, Eigen::Vector3d start, Eigen::Vector3d inter, Eigen::Vector3d end);





    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    void updateParams();

  private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    std::shared_ptr<bezier_kick::KickIK> ik_;

    std::shared_ptr<kicking::ParamListener> param_listener_;
    kicking::Params config_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    uint8_t joint_size_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> sensor_key_;
    std::unordered_map<std::string, double> sensor_state_;
    std::unordered_map<std::string, uint8_t> joint_map_;
    std::unordered_map<std::string, double> joint_command_;
    std::unordered_map<std::string, double> joint_state_;

    // MoveIt!
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr current_state_;

    enum KickState
    {
      Kick_Start_Shift = 0,
      Kick_Lift = 1,
      Kick_Swing = 2,
      Kick_Retract = 3,
      Kick_Landing = 4,
      Kick_Finish_Shift = 5,
      Kick_Finished = 6,
    };

    enum
    {
      KickingDisable = 0,
      KickingEnable = 1,
      KickingInitPose = 2,
      KickingReady = 3,
    };
    enum
    {
      Right_Kick = 1,
      Left_Kick = 2,
    };

    int control_cycle_msec_;
    std::string config_path_;
    bool real_running_;
    int kicking_state_;


    /* IMU (Inertial Measurement Unit) */
    Eigen::Vector3d imu;
    Eigen::Quaterniond imu_orientation;
    Eigen::Vector3d m_imuGyr;
    Eigen::Vector3d m_imuOri;
    Eigen::Vector3d m_imuAcc;
    Eigen::MatrixXd rotIMU;

    //ROS Publisher and Subscriber
    void kickStatus(const std::string& command);

    Eigen::MatrixXd target_position_;
    Eigen::MatrixXd current_position_;
    Eigen::MatrixXd init_position_;
    Eigen::MatrixXi joint_axis_direction_;
    std::map<std::string, int> joint_table_;

    bool firstExc;
    double firstTimeExc;
    double timeExc;

    int init_pose_count_;
    Eigen::MatrixXd calc_joint_tra_;

    std::string op3_joint[20], joint[18], offset_path_;
    double offset[18], op3_offset[20];

    bool m_active;
    bool m_start;
    int m_kick_mode;
    int m_kick_state;
    double m_time_start;
    double m_time_now;
    double m_time_state[6];
    double phase;

    double ankleShift_roll;
    double ankleShift_pitch;
    double kneeShift_pitch;
    double hipShift_pitch;
    double ankleShift[2];
    double kneeShift;
    double hipShift;
    double ankleImuParamRoll[4];
    double ankleImuParamPitch[4];
    double kneeImuParamPitch[4];
    double hipImuParamPitch[4];
    double gyroFactor;


    Eigen::Matrix<double, 18, 1> getJoints;
    Eigen::Vector3d actual_torso;
    Eigen::Matrix<double, 6, 1> L_Leg, R_Leg, Torso;
    Eigen::Matrix<double, 6, 1> uL_Leg, uR_Leg, uTorso;

    Eigen::Affine3d BASE;
    Eigen::Affine3d R_HIP, R_KNEE, R_ANKLE, R_FOOT, m_R_FOOT;
    Eigen::Affine3d L_HIP, L_KNEE, L_ANKLE, L_FOOT, m_L_FOOT;
    Eigen::Vector3d L_FOOT_WORLD, R_FOOT_WORLD;
    Eigen::Vector3d L_ROT_WORLD, R_ROT_WORLD;


    // Eigen::VectorXd L_FOOT_WORLD, R_FOOT_WORLD;
    Eigen::VectorXd LFootWorld, RFootWorld;
    int IndexSupport;

    void setValue();

    double procFunc(double value, double deadband, double maxValue);
    void legStabilization();
    void estimateZMP();
    Eigen::Vector3d bezierCurve2(double phase, Eigen::Vector3d start, Eigen::Vector3d end);
    Eigen::Vector3d bezierCurve3(double phase, Eigen::Vector3d start, Eigen::Vector3d inter, Eigen::Vector3d end);
    void motion_arms();
    void forwardKinematic();
  };

}  // namespace bezier_kick

#endif  // BEZIER_KICK__BEZIER_KICK_HPP_
