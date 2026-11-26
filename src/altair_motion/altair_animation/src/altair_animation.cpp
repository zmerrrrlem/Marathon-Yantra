#include "altair_animation/altair_animation.hpp"
#include "controller_interface/helpers.hpp"
#include <stdexcept>
#include <filesystem>
#include <algorithm>
#include "altair_msgs/srv/altair_animation.hpp" 

namespace altair_animation
{
  controller_interface::CallbackReturn AltairAnimation::on_init()
  {
    // Initialize animation service
    animation_service_ = nullptr;
    trajectory_index_ = 0;
    current_animation_ = "";
    need_smooth_transition_ = false;
    transition_complete_ = false;
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AltairAnimation::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    dxls_index_ = {
      {"r_sho_pitch", 1}, {"l_sho_pitch", 2}, {"r_sho_roll", 3}, {"l_sho_roll", 4},
      {"r_el", 5}, {"l_el", 6}, {"r_hip_yaw", 7}, {"l_hip_yaw", 8},
      {"r_hip_roll", 9}, {"l_hip_roll", 10}, {"r_hip_pitch", 11}, {"l_hip_pitch", 12},
      {"r_knee", 13}, {"l_knee", 14}, {"r_ank_roll", 15}, {"l_ank_roll", 16},
      {"r_ank_pitch", 17}, {"l_ank_pitch", 18}, {"head_tilt", 19}, {"head_pan", 20}
    };

    joint_names_ = {
        "head_pan", "head_tilt", "l_el", "l_sho_roll", "l_sho_pitch",
        "r_el", "r_sho_roll", "r_sho_pitch", "l_ank_roll", "l_ank_pitch",
        "l_knee", "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "r_ank_roll",
        "r_ank_pitch", "r_knee", "r_hip_pitch", "r_hip_roll", "r_hip_yaw"
    };
    joint_size_ = joint_names_.size();



    std::string a = ament_index_cpp::get_package_share_directory("altair_animation");
    std::string data_play = a + "/data/hello.yaml";


    mot_cont_ = 50;


    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AltairAnimation::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    // Handle controller restarts and dynamic parameter updating
    // ...

    
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AltairAnimation::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AltairAnimation::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    // Remove the service
    animation_service_.reset();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AltairAnimation::on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for shutdown transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn AltairAnimation::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    // Callback function for erroneous transition
    // ...
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration AltairAnimation::command_interface_configuration() const
  {
    // claim command interface only for the specified joints
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& [joint_name, id] : dxls_index_)
    {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration AltairAnimation::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    return state_interfaces_config;
  }

  std::vector<double> AltairAnimation::minimum_jerk_trajectory(double init_pos, double target_pos, double total_time, double dt) {
    std::vector<double> trajectory;
    double d = total_time;

    // convert to radians
    init_pos = init_pos * M_PI / 180;
    target_pos = target_pos * M_PI / 180;

    // Calculate the trajectory for each time step
    for (double t = 0; t < d; t += dt) {
      double normalized_time = t / d;
      double joint_position = init_pos + (target_pos - init_pos) *
        (10 * std::pow(normalized_time, 3) - 15 * std::pow(normalized_time, 4) + 6 * std::pow(normalized_time, 5));
      trajectory.push_back(joint_position);
    }

    return trajectory;
  }

  void AltairAnimation::move_smoother(double& current, double target) {
    // Arguments: current: reference to current servo position, target: target servo position
    // Optional steps parameter to control smoothing rate (higher = smoother but slower)
    if (mot_cont_ < 1) {
      current = target;
    }
    else {
      current += (target - current) / mot_cont_;
    }
  }

  controller_interface::return_type AltairAnimation::update(const rclcpp::Time& time, const rclcpp::Duration& period)
  {
    // Example waypoints - in a real implementation these would come from loaded animations
    // Three waypoints for each joint (just random values as examples)
    static std::vector<std::vector<double>> waypoints = {
      {0.1, 0.7, 0},    // waypoint set 1 for joint 0
      {-0.2, 0.4, 0.1},   // waypoint set 1 for joint 1
      {0.3, 1, -0.3},   // waypoint set 1 for joint 2
      // ... add for all joints
    };
    
    static size_t current_waypoint = 0;
    static std::vector<double> current_positions(joint_size_, 0.0);
    static bool initialized = false;
    static bool waypoint_reached = false;
    
    // Initialize current positions on first run
    if (!initialized) {
      for (size_t i = 0; i < joint_size_; ++i) {
        // In a real implementation, you would get the actual current positions
        // This is just placeholder code assuming we start at position 0
        current_positions[i] = 0.0;
      }
      initialized = true;
      mot_cont_ = 50; // Reset smoothing factor
    }
    
    // Check if we need to move to the next waypoint
    if (waypoint_reached && current_waypoint < 2) { // We have 3 waypoints (0,1,2)
      current_waypoint++;
      waypoint_reached = false;
      mot_cont_ = 50; // Reset smoothing factor for new waypoint
      RCLCPP_INFO(get_node()->get_logger(), "Moving to waypoint %zu", current_waypoint + 1);
    }

    // Decrement motion control counter
    if (mot_cont_ > 0) {
      --mot_cont_;
    } else {
      waypoint_reached = true;
      if (current_waypoint >= 2) {
        done_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Animation complete");
      }
    }

    // Apply smooth movement for each joint towards current waypoint
    bool all_joints_at_target = true;
    for (size_t i = 0; i < joint_size_; ++i) {
      // Get target from waypoints (ensure we have data for this joint)
      double target = 0.0;
      if (i < waypoints.size() && current_waypoint < waypoints[i].size()) {
        target = waypoints[i][current_waypoint];
      }
      
      // Apply smooth movement
      move_smoother(current_positions[i], target);
      
      // Check if this joint has reached target (with small epsilon)
      double epsilon = 0.01;
      all_joints_at_target = all_joints_at_target && 
                           (std::abs(current_positions[i] - target) < epsilon);
      
      // Set joint command to smoothed position
      if (i < joint_size_ && i < command_interfaces_.size()) {
        command_interfaces_[i].set_value(current_positions[i]);
      }
    }
    
    // If all joints are close enough to target, mark waypoint as reached
    if (all_joints_at_target) {
      waypoint_reached = true;
    }
    
    return controller_interface::return_type::OK;
  }

  // Service callback to handle animation requests
  void AltairAnimation::handle_animation_service(
    const std::shared_ptr<altair_msgs::srv::AltairAnimation::Request> request,
    std::shared_ptr<altair_msgs::srv::AltairAnimation::Response> response)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Received animation request: %s", request->animation_name.c_str());

    std::string anim_name = request->animation_name;
  }


}  // namespace altair_animation
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(altair_animation::AltairAnimation, controller_interface::ControllerInterface)