#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <ncurses.h>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <random>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <yaml-cpp/yaml.h>

#include "std_srvs/srv/set_bool.hpp"
#include "dynamixel_msgs/srv/set_ping.hpp"
#include "dynamixel_msgs/srv/set_torque.hpp"

namespace offset_editor {

  struct Joint {
    int id;
    std::string name;
    bool enabled;
    float position;
    float offset;
  };

  class OffsetEditor : public rclcpp::Node {

  public:

    OffsetEditor();
    ~OffsetEditor();

    void draw_UI();
    bool check_servo_accessible(uint8_t id);
    void update_positions_only();
    void toggle_torque();
    void set_init_pose();
    void handle_input();
    void load_yaml();
    void save_yaml();

  private:
    rclcpp::Node::SharedPtr node_;

    Joint joint_;
    std::vector<Joint> legs_;
    std::vector<Joint> arms_;
    std::vector<Joint> head_;
    std::unordered_map<std::string, int> joint_id_map_;
    std::set<uint8_t> accessible_servos_;

    std::vector<std::vector<Joint>*> categories_;
    std::vector<std::string> categoryNames_;
    int selectedCategoryIndex_;
    int selectedJoint_;

    rclcpp::Client<dynamixel_msgs::srv::SetTorque>::SharedPtr set_torque_client_;
    rclcpp::Client<dynamixel_msgs::srv::SetPing>::SharedPtr set_ping_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_init_pose_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    void ping_all_servos();
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  };


} // offset_editor
