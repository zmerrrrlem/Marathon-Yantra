#ifndef microcontroller_broadcaster__microcontroller_broadcaster_HPP_
#define microcontroller_broadcaster__microcontroller_broadcaster_HPP_

#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <chrono>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <fstream>
#include <tuple>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "semantic_components/imu_sensor.hpp"

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

#include <microcontroller_broadcaster/microcontroller_broadcaster_parameters.hpp> 

namespace microcontroller_broadcaster
{

  class MicrocontrollerBroadcaster : public controller_interface::ControllerInterface

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
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;

    using StatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr sensor_state_publisher_;
    std::unique_ptr<StatePublisher> realtime_sensor_state_publisher_;

    using ButtonPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Int16>;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr button_state_publisher_;
    std::unique_ptr<ButtonPublisher> realtime_button_state_publisher_;

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, uint8_t> joint_map_;

    bool enable_ = true;

    std::vector<std::string> dxls_key_;
    std::vector<std::string> sensor_key_;
    std::unordered_map<std::string, double> sensor_state_;
    std::unordered_map<std::string, uint8_t> dxls_index_;
    std::unordered_map<std::string, double> dxls_command_;
    std::unordered_map<std::string, double> dxls_state_;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> chatter_subsriber_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> chatter_publisher_;
    void chatterCb(const std_msgs::msg::String::SharedPtr msg);
  };

}  // namespace microcontroller_broadcaster

#endif  // microcontroller_broadcaster__microcontroller_broadcaster_HPP_
