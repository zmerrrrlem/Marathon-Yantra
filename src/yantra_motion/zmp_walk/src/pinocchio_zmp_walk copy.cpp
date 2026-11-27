#include "zmp_walk/zmp_walk.hpp"
#include "controller_interface/helpers.hpp"
#include <stdexcept>
#include <chrono>
#include <memory>
#include <string>
using namespace std::chrono_literals;

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

    auto temporary_pinocchio_loader = std::make_shared<rclcpp::Node>("temporary_pinocchio_loader");
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(temporary_pinocchio_loader, "robot_state_publisher");
    urdf_string = parameters_client->get_parameter<std::string>("robot_description");

    if (urdf_string.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "robot_description is empty");
      return CallbackReturn::ERROR;
    }

    // === Build Pinocchio model_ ===

    try {
      pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF for Pinocchio: %s", e.what());
      return CallbackReturn::SUCCESS;
    }

    data_ = pinocchio::Data(model_);
    RCLCPP_INFO(node_->get_logger(), "Pinocchio model_ loaded: %s", model_.name.c_str());
    RCLCPP_INFO(node_->get_logger(), "model_ has: %d DOFs", model_.nq);

    q_ = neutral(model_);                   // Initial configuration
    v_ = Eigen::VectorXd::Zero(model_.nv);  // Zero velocity
    a_ = Eigen::VectorXd::Zero(model_.nv);  // Zero acceleration


    joint_names_ = { "l_ank_roll", "l_ank_pitch", "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "r_ank_roll", "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw" };
    for (uint8_t i = 0; i < joint_names_.size(); i++) {
      joint_map_[joint_names_[i]] = i;
    }

    state_observer_ = std::make_shared<StateObserver>();

    return CallbackReturn::SUCCESS;
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

  // Convert quaternion + translation to Eigen::Isometry3d
  Eigen::Isometry3d toIsometry(double tx, double ty, double tz,
    double qx, double qy, double qz, double qw)
  {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = q.normalized().toRotationMatrix();
    iso.translation() = Eigen::Vector3d(tx, ty, tz);
    return iso;
  }

  // Simple damped least squares inverse kinematics for a frame
  bool solveIK(const pinocchio::Model& model, pinocchio::Data& data,
    Eigen::VectorXd& q,
    const std::vector<pinocchio::JointIndex>& joint_ids,
    pinocchio::FrameIndex target_frame,
    const Eigen::Isometry3d& target_pose,
    int max_iters = 100, double tol = 1e-2)
  {
    double lambda = 1e-4;
    for (int iter = 0; iter < max_iters; ++iter)
    {
      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      Eigen::Isometry3d M = Eigen::Isometry3d::Identity();
      M.linear() = data.oMf[target_frame].rotation();
      M.translation() = data.oMf[target_frame].translation();
      Eigen::Matrix<double, 6, 1> err;
      err.head<3>() = target_pose.translation() - M.translation();

      Eigen::Matrix3d R_err = target_pose.linear() * M.linear().transpose();
      Eigen::AngleAxisd aa(R_err);
      err.tail<3>() = aa.axis() * aa.angle();

      if (err.norm() < tol) return true;

      Eigen::MatrixXd J(6, model.nv);
      pinocchio::computeFrameJacobian(model, data, q, target_frame,
        pinocchio::ReferenceFrame::WORLD, J);

      // Build reduced Jacobian for selected joints
      Eigen::MatrixXd J_reduced(6, joint_ids.size());
      for (size_t i = 0; i < joint_ids.size(); ++i)
        J_reduced.col(i) = J.col(joint_ids[i]);

      // Damped least squares solution
      Eigen::MatrixXd JtJ = J_reduced.transpose() * J_reduced;
      Eigen::VectorXd dq = (JtJ + lambda * Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols()))
        .ldlt().solve(J_reduced.transpose() * err);

      for (size_t i = 0; i < joint_ids.size(); ++i)
        q[joint_ids[i]] += dq[i];
    }
    return false;
  }

  controller_interface::return_type ZMPWalk::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {

    // Is left support foot: false
// [quintic_walk]: Support foot to trunk translation: x=0.000000, y=0.052784, z=0.280000
// [quintic_walk]: Support foot to trunk rotation: x=0.000000, y=0.049979, z=0.000000, w=0.998750
// [quintic_walk]: Support foot to flying foot translation: x=0.000000, y=0.130000, z=0.024620
// [quintic_walk]: Support foot to flying foot rotation: x=0.000000, y=0.000000, z=0.000000, w=1.000000
// [quintic_walk]: Trunk to support foot translation: x=0.027953, y=-0.052784, z=-0.278601
// [quintic_walk]: Trunk to support foot rotation: x=0.000000, y=-0.049979, z=0.000000, w=0.998750
// [quintic_walk]: Trunk to flying foot translation: x=0.025495, y=0.077216, z=-0.254104
// [quintic_walk]: Trunk to flying foot rotation: x=0.000000, y=-0.049979, z=0.000000, w=0.998750


    forwardKinematics(model_, data_, q_);
    updateFramePlacements(model_, data_);

    /// Define transforms
    Eigen::Isometry3d trunk_to_support = toIsometry(
      0.027953, -0.052784, -0.278601,
      0.0, -0.049979, 0.0, 0.998750);

    Eigen::Isometry3d trunk_to_flying = toIsometry(
      0.025495, 0.077216, -0.254104,
      0.0, -0.049979, 0.0, 0.998750);

    Eigen::Isometry3d l_sole_goal = toIsometry(
      0.215274, 0.116889, 0.0378277,
      0.00501981, -0.0969865, 0.0226112, 0.995016);

    // Compute goal for flying foot in world frame:
    // world --> support_foot --> trunk --> flying
    pinocchio::FrameIndex r_sole_id = model_.getFrameId("r_sole");
    pinocchio::FrameIndex l_sole_id = model_.getFrameId("l_sole");

    // Print position and orientation of l_sole
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    // Extract position of l_sole
    Eigen::Vector3d l_sole_position = data_.oMf[l_sole_id].translation();

    // Extract orientation of l_sole as rotation matrix
    Eigen::Matrix3d l_sole_rotation = data_.oMf[l_sole_id].rotation();

    // Convert rotation matrix to quaternion for more readable output
    Eigen::Quaterniond l_sole_quaternion(l_sole_rotation);

    // Print the information
    std::cout << "Left sole position: ["
      << l_sole_position.x() << ", "
      << l_sole_position.y() << ", "
      << l_sole_position.z() << "]" << std::endl;
    std::cout << "Left sole orientation (quaternion): ["
      << l_sole_quaternion.x() << ", "
      << l_sole_quaternion.y() << ", "
      << l_sole_quaternion.z() << ", "
      << l_sole_quaternion.w() << "]" << std::endl;

    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    Eigen::Isometry3d oM_r_sole = Eigen::Isometry3d::Identity();
    oM_r_sole.linear() = data_.oMf[r_sole_id].rotation();
    oM_r_sole.translation() = data_.oMf[r_sole_id].translation();

    Eigen::Isometry3d oM_trunk = oM_r_sole * trunk_to_support;
    Eigen::Isometry3d oM_l_sole_goal = oM_trunk * trunk_to_flying;

    // Define joint subset for left leg
    std::vector<std::string> left_leg_joint_names = {
        "l_hip_yaw", "l_hip_roll", "l_hip_pitch",
        "l_knee", "l_ank_pitch", "l_ank_roll"
    };
    std::vector<pinocchio::JointIndex> left_leg_joint_ids;
    for (const auto& name : left_leg_joint_names)
      left_leg_joint_ids.push_back(model_.getJointId(name));

    // Solve IK
    // Start the timer
    auto start = std::chrono::high_resolution_clock::now();
    
    // Call solveIK
    bool success = solveIK(model_, data_, q_, left_leg_joint_ids, l_sole_id, l_sole_goal);
    
    // Stop the timer
    auto end = std::chrono::high_resolution_clock::now();
    
    // Calculate duration
    std::chrono::duration<double, std::milli> duration = end - start;
    
    // Print execution time
    std::cout << "IK solving took " << duration.count() << " ms" << std::endl;

    if (success) {
      std::cout << "IK solved! Joint values:" << std::endl;
      for (const auto& joint_name : joint_names_) {
        pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
        std::cout << joint_name << ": " << q_[model_.joints[joint_id].idx_q()] << std::endl;
      }
    }
    else
      std::cout << "IK failed!" << std::endl;

    // === INVERSE DYNAMICS ===
    Eigen::VectorXd tau = rnea(model_, data_, q_, v_, a_);
    std::cout << "Inverse Dynamics torque:\n" << tau.transpose() << std::endl;


    return controller_interface::return_type::OK;
  }

}  // namespace zmp_walk
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(zmp_walk::ZMPWalk, controller_interface::ControllerInterface)