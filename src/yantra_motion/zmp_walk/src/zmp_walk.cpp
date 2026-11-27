#include "zmp_walk/zmp_walk.hpp"
#include "controller_interface/helpers.hpp"
#include <stdexcept>
#include <chrono>
#include <memory>
#include <string>
using namespace std::chrono_literals;
using drake::math::DiscreteAlgebraicRiccatiEquation;

namespace zmp_walk
{
  controller_interface::CallbackReturn ZMPWalk::on_init()
  {
    try
    {
      node_ = get_node();
      param_listener_ = std::make_shared<walking::ParamListener>(get_node());
      params_ = param_listener_->get_params();
      RCLCPP_INFO(get_node()->get_logger(), "on_init");
    }
    catch (const std::exception& e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ZMPWalk::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    std::string urdf_string;


    // Subscriber and Publisher
    // Subscriber for cmd_vel
    cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/yantra/cmd_vel", 10, std::bind(&ZMPWalk::cmd_vel_cb, this, std::placeholders::_1));


    joint_names_ = { "l_ank_roll", "l_ank_pitch", "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "r_ank_roll", "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw" };
    for (uint8_t i = 0; i < joint_names_.size(); i++) {
      joint_map_[joint_names_[i]] = i;
    }

    state_observer_ = std::make_shared<StateObserver>();

    return CallbackReturn::SUCCESS;
  }

  double ZMPWalk::get_phase()
  {
    double current_ros_time = get_node()->get_clock()->now().seconds();

    // Initialize t on first call
    if (t == 0.0) {
      t = current_ros_time;
      last_time = current_ros_time;
    }

    phase_ = (current_ros_time - last_time) / step_period_;
    if (phase_ > 1) {
      new_step_ = true;
      phase_ = 0;
      last_time = last_time + step_period_;
    }
    return phase_;
  }


  // Foot placement constraints (in meters)
  double min_foot_distance_x = 0.10;  // Minimum horizontal distance between feet
  double max_foot_distance_x = 0.30;  // Maximum horizontal distance between feet
  double min_foot_distance_y = 0.05;  // Minimum lateral distance between feet
  double max_foot_distance_y = 0.20;  // Maximum lateral distance between feet

  Eigen::Vector2d ZMPWalk::generate_next_footstep_position() {
    // Calculate step length based on cmd_vel
    double step_length = std::max(min_foot_distance_x, std::min(x_move_ * step_period_, max_foot_distance_x));
    double lateral_step = std::max(min_foot_distance_y / 2, std::min(std::abs(y_move_) * step_period_, max_foot_distance_y / 2));

    // Apply lateral step direction
    if (y_move_ < 0) lateral_step = -lateral_step;

    // Calculate forward movement components with rotation
    double dx = step_length * std::cos(a_move_ * step_period_);
    double dy = step_length * std::sin(a_move_ * step_period_) + lateral_step;

    return Eigen::Vector2d(dx, dy);
  }


  // Create system matrices for digital controller
  void ZMPWalk::create_system(double dT, double zc, double g, Eigen::Matrix3d& A, Eigen::Vector3d& B, Eigen::RowVector3d& C)
  {
    // State transition matrix A
    A << 1, dT, dT* dT / 2,
      0, 1, dT,
      0, 0, 1;

    // Input matrix B
    B << std::pow(dT, 3) / 6,
      std::pow(dT, 2) / 2,
      dT;

    // Output matrix C
    C << 1, 0, -zc / g;
  }

  // Create controller for tracking control problem minimizing performance J
  void ZMPWalk::create_controller(const Eigen::Matrix3d& A, const Eigen::Vector3d& B, const Eigen::RowVector3d& C,
    double r, double q, int N, Eigen::RowVector3d& K, Eigen::VectorXd& Fs)
  {
    // R and Q matrices
    Eigen::MatrixXd R = r * Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd Q = q * C.transpose() * C;

    // Solve discrete-time algebraic Riccati equation using Drake
    Eigen::MatrixXd P = DiscreteAlgebraicRiccatiEquation(A, B, Q, R);

    // Compute K
    Eigen::MatrixXd tmp = (R + B.transpose() * P * B).inverse() * B.transpose();
    K = tmp * P * A;

    // Compute Fs
    Fs.resize(N);
    Eigen::MatrixXd pre = tmp;
    Eigen::MatrixXd AcT = (A - B * K).transpose();
    for (int n = 0; n < N; ++n)
    {
      Fs(n) = (pre * C.transpose() * q)(0, 0);
      pre = pre * AcT;
    }
  }

  // Update state for ZMP walking
  void update_state(const Eigen::Matrix3d& A, const Eigen::Vector3d& B, const Eigen::RowVector3d& C,
    const Eigen::Vector3d& X, double U,
    Eigen::Vector3d& X_next, double& P_k)
  {
    X_next = A * X + B * U;
    P_k = C * X;
  }

      // Solve system for current phase only
      struct StepSolution {
        Eigen::Vector3d state;
        double zmp;
      };

      StepSolution ZMPWalk::solve_system_step(
        const Eigen::Matrix3d& A, 
        const Eigen::Vector3d& B, 
        const Eigen::RowVector3d& C,
        const Eigen::RowVector3d& K, 
        const Eigen::VectorXd& Fs,
        const Eigen::VectorXd& pref, 
        int current_index,
        const Eigen::Vector3d& X)
      {
        // Prepare reference window
        int N = Fs.size();
        Eigen::VectorXd pref_window(N);
        for (int i = 0; i < N; ++i) {
          int idx = std::min(current_index + i, static_cast<int>(pref.size() - 1));
          pref_window(i) = pref(idx);
        }
        
        // Calculate control input using current state and reference trajectory
        double U = -(K * X)(0) + (Fs.dot(pref_window));
        
        // Update state and ZMP
        Eigen::Vector3d X_next;
        double P_k;
        update_state(A, B, C, X, U, X_next, P_k);
        
        return {X_next, P_k};
      }

  // Implementation of update function
  controller_interface::return_type ZMPWalk::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {

    double phase = get_phase();



    return controller_interface::return_type::OK;
  }

  void ZMPWalk::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    x_move_ = msg->linear.x;
    y_move_ = msg->linear.y;
    a_move_ = msg->angular.z;
  }

  controller_interface::CallbackReturn ZMPWalk::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    // ...
    std::cout << "activate zmp_walk" << std::endl;
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ZMPWalk::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ZMPWalk::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for cleanup transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ZMPWalk::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ZMPWalk::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration ZMPWalk::command_interface_configuration() const
  {

    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration ZMPWalk::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    return state_interfaces_config;
  }

}  // namespace zmp_walk
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zmp_walk::ZMPWalk, controller_interface::ControllerInterface)