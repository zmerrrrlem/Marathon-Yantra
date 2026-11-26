#ifndef ALTAIR_ANIMATION__ALTAIR_ANIMATION_HPP_
#define ALTAIR_ANIMATION__ALTAIR_ANIMATION_HPP_

#include <chrono>
#include <thread>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <tuple>
#include <unordered_map>

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

#include <yaml-cpp/yaml.h>

#include "altair_animation_parameters.hpp"
#include "altair_msgs/srv/altair_animation.hpp"

namespace altair_animation
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Animation data structure to store all information for each animation
  struct Animation {
    std::string name;                                        // Animation file name
    double duration;                                         // Total duration specified in YAML
    std::unordered_map<std::string, std::vector<double>> waypoints;  // Raw joint waypoints from YAML
    std::unordered_map<int, std::vector<double>> trajectories;       // Processed trajectories by joint ID
  };

  class AltairAnimation : public controller_interface::ControllerInterface
  {
  public:
    AltairAnimation() = default;
    ~AltairAnimation() = default;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  private:

    uint8_t joint_size_;
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, uint8_t> joint_map_;

    void move_smoother(double& current, double target);
    std::vector<double> minimum_jerk_trajectory(double init_pos, double target_pos, double total_time, double dt);

    uint8_t mot_cont_;
    bool done_ = false;

    // Service callback to handle animation requests
    void handle_animation_service(
      const std::shared_ptr<altair_msgs::srv::AltairAnimation::Request> request,
      std::shared_ptr<altair_msgs::srv::AltairAnimation::Response> response);

    // Helper function to load animation from file
    bool load_animation(const std::string& file_name);

    // Helper function to update current positions from state interfaces
    void update_current_positions();

    // Helper function to create transition trajectory
    void create_transition_trajectory(const std::string& anim_name);

    // Member variables
    std::unordered_map<std::string, uint8_t> dxls_index_;
    std::unordered_map<int, double> current_positions_;     // Current joint positions
    std::unordered_map<std::string, Animation> loaded_animations_;  // All loaded animations
    std::string current_animation_;                         // Currently active animation
    std::string next_animation_;                            // Animation to switch to after transition
    size_t trajectory_index_ = 0;                           // Current position in trajectory
    bool need_smooth_transition_ = false;                   // Flag for smooth transitions
    bool transition_complete_ = false;                      // Flag for transition completion

    // Service server
    rclcpp::Service<altair_msgs::srv::AltairAnimation>::SharedPtr animation_service_;

    // Paths
    std::string package_path_;
    std::string data_dir_;
  };
} // namespace altair_animation
#endif  // ALTAIR_ANIMATION__ALTAIR_ANIMATION_HPP_