#ifndef ZMPWALK__ZMPWALK_HPP_
#define ZMPWALK__ZMPWALK_HPP_

#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <fstream>
#include <tuple>

// Drake library for Riccati equation solving
#include <drake/math/discrete_algebraic_riccati_equation.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
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
#include "geometry_msgs/msg/twist.hpp"

#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include "zmp_walk/state_observer.hpp"
#include <zmp_walk/zmp_walk_parameters.hpp>

namespace zmp_walk
{

  class ZMPWalk : public controller_interface::ControllerInterface

  {
  public:
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



  private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::shared_ptr<walking::ParamListener> param_listener_;
    walking::Params params_;

    std::shared_ptr<zmp_walk::StateObserver> state_observer_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd a_;

    pinocchio::Model model_;
    pinocchio::Data data_;

    double get_phase();
    
    // ZMP control functions
    Eigen::Vector2d generate_next_footstep_position();
    void create_system(double dT, double zc, double g, Eigen::Matrix3d& A, Eigen::Vector3d& B, Eigen::RowVector3d& C);
    void create_controller(const Eigen::Matrix3d& A, const Eigen::Vector3d& B, const Eigen::RowVector3d& C,
                          double r, double q, int N, Eigen::RowVector3d& K, Eigen::VectorXd& Fs);
    
    bool new_step_ = false;

    // Walking phase and timing variables
    double phase_ = 0.0;
    double step_period_ = 0.5;  // Default step period in seconds
    double last_time = 0.0;
    double t = 0.0;  // Current time reference

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, uint8_t> joint_map_;

    bool enable_ = true;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;

    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    float x_move_, y_move_, a_move_;

    std::vector<std::string> dxls_key_;
    std::vector<std::string> sensor_key_;
    std::unordered_map<std::string, double> sensor_state_;
    std::unordered_map<std::string, uint8_t> dxls_index_;
    std::unordered_map<std::string, double> dxls_command_;
    std::unordered_map<std::string, double> dxls_state_;

  };

}  // namespace zmp_walk

#endif  // ZMPWALK__ZMPWALK_HPP_
