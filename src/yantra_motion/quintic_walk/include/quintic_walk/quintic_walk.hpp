#ifndef QUINTIC_WALK__QUINTIC_WALK_HPP_
#define QUINTIC_WALK__QUINTIC_WALK_HPP_

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
#include <moveit/robot_model/robot_model.hpp>
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
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "biped_interfaces/msg/phase.hpp"
#include "yantra_msgs/msg/foot_pressure.hpp"
#include "yantra_msgs/msg/joint_command.hpp"
#include "yantra_msgs/msg/robot_control_state.hpp"
#include "quintic_walk/walk_node.hpp"
#include "quintic_walk/walk_engine.hpp"
#include "quintic_walk/walk_ik.hpp"
#include "quintic_walk/walk_stabilizer.hpp"
#include "quintic_walk/walk_visualizer.hpp"
#include "quintic_splines/abstract_ik.hpp"


#include "quintic_walk_parameters.hpp"

namespace quintic_walk
{

  class QuinticWalk : public controller_interface::ControllerInterface

  {
  public:
    // QuinticWalk();

    yantra_msgs::msg::JointCommand step(double dt);
    yantra_msgs::msg::JointCommand step(double dt, geometry_msgs::msg::Twist::SharedPtr cmdvel_msg, sensor_msgs::msg::Imu::SharedPtr imu_msg, sensor_msgs::msg::JointState::SharedPtr jointstate_msg);
    geometry_msgs::msg::PoseArray step_open_loop(double dt, geometry_msgs::msg::Twist::SharedPtr cmdvel_msg);
    geometry_msgs::msg::Pose get_right_foot_pose();
    geometry_msgs::msg::Pose get_left_foot_pose();
    yantra_msgs::msg::JointCommand initial_position(double dt);

    /**
     * Reset everything to initial idle state.
     */
    void reset();
    /**
     * Updates the parameters of the walking after a parameter change.
     */
    void updateParams();
    /**
     * Reset walk to any given state. Necessary for using this as reference in learning.
     */
    void reset(WalkState state, double phase, geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool reset_odometry);
    /**
     * Initialize internal WalkEngine to correctly zeroed, usable state
     */
    void initializeEngine();
    /**
     * Sets the current state of the robot
     * @param msg The current state
     */
    void robotStateCb(yantra_msgs::msg::RobotControlState::SharedPtr msg);
    WalkEngine* getEngine();
    WalkIK* getIk();
    moveit::core::RobotModelPtr* get_kinematic_model();
    nav_msgs::msg::Odometry getOdometry();

    rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& parameters);

    void publish_debug();
    rclcpp::TimerBase::SharedPtr startTimer();
    double getTimerFreq();
    void move_smoother(double &buffer, double current, double target);
    // Safe access to the claimed interfaces by joint name. Unknown names are reported (throttled) and skipped.
    bool readJointState(const std::string& joint_name, double& value);
    bool writeJointCommand(const std::string& joint_name, double value);


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
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    void checkPhaseRestAndReset();
    std::array<double, 4> get_step_from_vel(geometry_msgs::msg::Twist::SharedPtr msg);
    void stepCb(geometry_msgs::msg::Twist::SharedPtr msg);
    void cmdVelCb(geometry_msgs::msg::Twist::SharedPtr msg);
    void kickCb(std_msgs::msg::Bool::SharedPtr msg);
    void jointStateCb(sensor_msgs::msg::JointState::SharedPtr msg);
    void imuCb(sensor_msgs::msg::Imu::SharedPtr msg);
    double getTimeDelta();
    

    // Declare parameter listener and struct from the generate_parameter_library
    std::shared_ptr<walking::ParamListener> param_listener_;
    // quintic_walk::ParamListener param_listener_;

    // Datastructure to hold all parameters, which is build from the schema in the 'parameters.yaml'
    walking::Params config_;
    walking::Params params_;

    std::shared_ptr<quintic_walk::WalkEngine> walk_engine_;
    std::shared_ptr<quintic_walk::WalkStabilizer> stabilizer_;
    std::shared_ptr<quintic_walk::WalkIK> ik_;
    std::shared_ptr<quintic_walk::WalkVisualizer> visualizer_;

    WalkRequest current_request_ = { .stop_walk = true };
    WalkRequest last_request_;

    bool first_run_ = true;

    double last_ros_update_time_ = 0;

    int robot_state_ = yantra_msgs::msg::RobotControlState::CONTROLLABLE;
    int current_support_foot_ = biped_interfaces::msg::Phase::DOUBLE_STANCE;

    WalkResponse current_response_;
    WalkResponse current_stabilized_response_;
    quintic_splines::JointGoals motor_goals_;

    nav_msgs::msg::Odometry odom_msg_;

    // MoveIt!
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr current_state_;

    double current_trunk_fused_pitch_ = 0;
    double current_trunk_fused_roll_ = 0;

    double current_fly_pressure_ = 0;
    double current_fly_effort_ = 0;

    std::optional<rclcpp::Time> last_imu_measurement_time_;
    double imu_y_acc = 0;

    bool got_new_goals_ = false;

    bool enable_ = true;

    bool init_position_;

    // motor step counter used for smooth movement, max unsigned 8 bit = 256
    uint8_t mot_cont_;
    double goal_pos_temp_[12] = {0};

    uint8_t joint_size_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> sensor_key_;
    std::unordered_map<std::string, double> sensor_state_;
    std::unordered_map<std::string, uint8_t> joint_map_;
    std::unordered_map<std::string, double> joint_command_;
    std::unordered_map<std::string, double> joint_state_;
    std::unordered_map<std::string, double> joint_axis_direction_;

    rclcpp::Publisher<yantra_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_support_;
    // Engine state (WalkState enum as int) for external nodes such as yantra_heading
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_walk_status_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_subscriber_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr step_sub_;
    rclcpp::Subscription<yantra_msgs::msg::RobotControlState>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kick_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  };

}  // namespace quintic_walk

#endif  // QUINTIC_WALK__QUINTIC_WALK_HPP_
