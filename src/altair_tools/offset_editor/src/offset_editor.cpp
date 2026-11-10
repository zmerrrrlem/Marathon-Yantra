#include <ncurses.h>
#include <vector>
#include <string>
#include <memory>
#include <iostream>

#include "offset_editor/offset_editor.hpp"


namespace offset_editor {

  OffsetEditor::OffsetEditor()
    : rclcpp::Node("offset_editor") {
    joint_id_map_ = {
        {"r_sho_pitch", 1}, {"l_sho_pitch", 2}, {"r_sho_roll", 3}, {"l_sho_roll", 4},
        {"r_el", 5}, {"l_el", 6}, {"r_hip_yaw", 7}, {"l_hip_yaw", 8},
        {"r_hip_roll", 9}, {"l_hip_roll", 10}, {"r_hip_pitch", 11}, {"l_hip_pitch", 12},
        {"r_knee", 13}, {"l_knee", 14}, {"r_ank_pitch", 15}, {"l_ank_pitch", 16},
        {"r_ank_roll", 17}, {"l_ank_roll", 18}, {"head_pan", 19}, {"head_tilt", 20}
    };
    legs_ = {
        {7, "r_hip_yaw", false, 0.0, 0.0},
        {9, "r_hip_roll", false, 0.0, 0.0},
        {11, "r_hip_pitch", false, 0.0, 0.0},
        {13, "r_knee", false, 0.0, 0.0},
        {15, "r_ank_pitch", false, 0.0, 0.0},
        {17, "r_ank_roll", false, 0.0, 0.0},
        {8, "l_hip_yaw", false, 0.0, 0.0},
        {10, "l_hip_roll", false, 0.0, 0.0},
        {12, "l_hip_pitch", false, 0.0, 0.0},
        {14, "l_knee", false, 0.0, 0.0},
        {16, "l_ank_pitch", false, 0.0, 0.0},
        {18, "l_ank_roll", false, 0.0, 0.0},
    };

    arms_ = {
        {1, "r_sho_pitch", false, 0.0, 0.0},
        {3, "r_sho_roll", false, 0.0, 0.0},
        {5, "r_el", false, 0.0, 0.0},
        {2, "l_sho_pitch", false, 0.0, 0.0},
        {4, "l_sho_roll", false, 0.0, 0.0},
        {6, "l_el", false, 0.0, 0.0},
    };

    head_ = {
        {19, "head_pan", false, 0.0, 0.0},
        {20, "head_tilt", false, 0.0, 0.0},
    };

    categories_ = { &legs_, &arms_, &head_ };
    categoryNames_ = { "legs", "arms", "head" };
    selectedCategoryIndex_ = 0;
    selectedJoint_ = 0;
    accessible_servos_ = {};

    set_torque_client_ = this->create_client<dynamixel_msgs::srv::SetTorque>("dynamixel_hardware/torque_enable");
    set_ping_client_ = this->create_client<dynamixel_msgs::srv::SetPing>("dynamixel_hardware/ping");
    set_init_pose_client_ = this->create_client<std_srvs::srv::SetBool>("dynamixel_hardware/set_init_pose");
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&OffsetEditor::jointStateCallback, this, std::placeholders::_1));

  }
  OffsetEditor::~OffsetEditor() {
  }

  void OffsetEditor::load_yaml() {
    // relative path to altair_description/urdf/offset.yaml
    std::string offset_file = "src/altair_misc/altair_description/urdf/offset.yaml";

    // Display file path for debugging
    mvprintw(18, 0, "Loading offsets from: %s", offset_file.c_str());
    refresh();
    napms(500);

    // Clear existing offsets
    for (auto& joint : legs_) joint.offset = 0.0;
    for (auto& joint : arms_) joint.offset = 0.0;
    for (auto& joint : head_) joint.offset = 0.0;

    try {
      // Verify file exists first
      std::ifstream fin(offset_file);
      if (!fin.good()) {
        mvprintw(19, 0, "Error: File not found or inaccessible");
        refresh();
        napms(2000);
        return;
      }
      fin.close();

      // Load YAML
      YAML::Node config = YAML::LoadFile(offset_file);

      // Check root node
      if (!config["offset"]) {
        mvprintw(19, 0, "Error: 'offset' key missing in YAML");
        refresh();
        napms(2000);
        return;
      }

      YAML::Node offset_node = config["offset"];
      int loaded_count = 0;

      // Load offsets
      for (const auto& joint_entry : offset_node) {
        std::string joint_name = joint_entry.first.as<std::string>();
        double offset_value = joint_entry.second.as<double>();

        // Find and update joint in all categories
        auto update_joint = [&](std::vector<Joint>& joints) {
          for (auto& joint : joints) {
            if (joint.name == joint_name) {
              joint.offset = offset_value;
              loaded_count++;
              mvprintw(20 + loaded_count, 0, "%-12s: %.5f",
                joint_name.c_str(), offset_value);
              break;
            }
          }
          };

        update_joint(legs_);
        update_joint(arms_);
        update_joint(head_);
      }

      mvprintw(19, 0, "Loaded %d offsets successfully", loaded_count);
      refresh();
      napms(1000);

    }
    catch (const YAML::ParserException& e) {
      mvprintw(19, 0, "YAML Syntax Error: %s", e.what());
      refresh();
      napms(2000);
    }
    catch (const YAML::BadFile& e) {
      mvprintw(19, 0, "File Error: %s", e.what());
      refresh();
      napms(2000);
    }
    catch (const std::exception& e) {
      mvprintw(19, 0, "Error: %s", e.what());
      refresh();
      napms(2000);
    }
  }

  void OffsetEditor::save_yaml() {
    // relative path to altair_description/urdf/offset.yaml
    std::string offset_file = "src/altair_misc/altair_description/urdf/offset.yaml";

    try {
      YAML::Node config;
      YAML::Node offset_node;

      // Save all joints from all categories
      for (auto category : categories_) {
        for (auto& joint : *category) {
          offset_node[joint.name] = joint.offset;
        }
      }

      config["offset"] = offset_node;

      std::ofstream fout(offset_file);
      fout << config;
      fout.close();

      mvprintw(20, 0, "Successfully saved offsets to %s", offset_file.c_str());
      refresh();
    }
    catch (const YAML::Exception& e) {
      mvprintw(20, 0, "Error saving YAML: %s", e.what());
      refresh();
    }
  }

  void OffsetEditor::draw_UI() {
    clear();

    // Title and category selection
    mvprintw(0, 0, "Select: [legs] [arms] [head]   [Current: %s]", categoryNames_[selectedCategoryIndex_].c_str());

    // Column headers with fixed positions
    mvprintw(2, 0, "Stat ID     Name        Enable    Pos    Offset");
    mvprintw(3, 0, "-------------------------------------------------------------");

    std::vector<Joint>& joints = *categories_[selectedCategoryIndex_];

    for (size_t i = 0; i < joints.size(); i++) {
      // Status column (fixed at 0)
      if (check_servo_accessible(joints[i].id)) {
        attron(COLOR_PAIR(1));
        mvprintw(4 + i, 0, "[O]");
        attroff(COLOR_PAIR(1));
      }
      else {
        attron(COLOR_PAIR(2));
        mvprintw(4 + i, 0, "[X]");
        attroff(COLOR_PAIR(2));
      }

      // ID column (fixed at 4)
      mvprintw(4 + i, 4, "%d", joints[i].id);

      // Name column (fixed at 8)
      mvprintw(4 + i, 8, "%-12s", joints[i].name.c_str());

      // Enable column (fixed at 22)
      mvprintw(4 + i, 26, "%s", joints[i].enabled ? "[X]" : "[ ]");

      // Position column (fixed at 30)
      mvprintw(4 + i, 34, "%.2f", joints[i].position);

      // double difference = joints[i].position - 0;
      // mvprintw(4 + i, 42, "%.2f", difference);

      // Offset column (fixed at 38)
      mvprintw(4 + i, 42, "%.2f", joints[i].offset);


      // Highlight selected row
      if (i == selectedJoint_) {
        mvchgat(4 + i, 0, -1, A_REVERSE, 0, nullptr);
      }
    }

    // Footer
    mvprintw(6 + joints.size(), 0, "[Up/Down]Navigate   [Left/Right]Adjust   [Space]Toggle    [Tab]Switch ");
    mvprintw(7 + joints.size(), 0, "[P]Ping   [i]Init   [R]Refresh           [S]Save          [ctrl+C]Quit");
    mvprintw(8 + joints.size(), 0, "Status: [O]=Online [X]=Offline");

    refresh();
  }

  void OffsetEditor::ping_all_servos() {
    if (!set_ping_client_->wait_for_service(std::chrono::milliseconds(100))) {
      // Service not available
      mvprintw(22, 0, "Error: Servo ping service not available");
      refresh();
      napms(2000);
      return;
    }

    // Get all servo IDs from all categories
    std::vector<uint8_t> all_ids;
    for (auto category : categories_) {
      for (const auto& joint : *category) {
        all_ids.push_back(joint.id);
      }
    }

    auto request = std::make_shared<dynamixel_msgs::srv::SetPing::Request>();
    request->id = all_ids;

    mvprintw(22, 0, "Pinging all servos...");
    refresh();

    auto future = set_ping_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      accessible_servos_.clear();
      for (const auto& id : response->result) {
        accessible_servos_.insert(id);
      }
      mvprintw(22, 0, "Ping successful. Accessible servos updated.");
    }
    else {
      mvprintw(22, 0, "Error: Failed to ping servos (timeout)");
    }

    refresh();
    napms(1000);
  }

  void OffsetEditor::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto update_position = [&](std::vector<Joint>& joints) {
        for (auto& joint : joints) {
          if (joint.name == msg->name[i]) {
            joint.position = msg->position[i];
            break;
          }
        }
        };

      update_position(legs_);
      update_position(arms_);
      update_position(head_);
    }
  }

  bool OffsetEditor::check_servo_accessible(uint8_t id) {
    return accessible_servos_.count(id) > 0;
  }

  void OffsetEditor::toggle_torque() {
    (*categories_[selectedCategoryIndex_])[selectedJoint_].enabled = !(*categories_[selectedCategoryIndex_])[selectedJoint_].enabled;
    if (!set_torque_client_->wait_for_service(std::chrono::milliseconds(100))) {
      // Service not available
      mvprintw(22, 0, "Error: Servo torque service not available");
      refresh();
      napms(2000);
      return;
    }

    auto request = std::make_shared<dynamixel_msgs::srv::SetTorque::Request>();
    request->id = { (*categories_[selectedCategoryIndex_])[selectedJoint_].id };
    request->enable = !(*categories_[selectedCategoryIndex_])[selectedJoint_].enabled;

    auto future = set_torque_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->result) {
        mvprintw(22, 0, "Torque %s for servo %d",
          (*categories_[selectedCategoryIndex_])[selectedJoint_].enabled ? "disabled" : "enabled",
          (*categories_[selectedCategoryIndex_])[selectedJoint_].id);
      }
      else {
        mvprintw(22, 0, "Error: Failed to toggle torque");
      }
    }
    else {
      mvprintw(22, 0, "Error: Failed to toggle torque (timeout)");
    }

    refresh();
    napms(1000);
  }

  void OffsetEditor::set_init_pose() {
    if (!set_init_pose_client_->wait_for_service(std::chrono::milliseconds(100))) {
      // Service not available
      mvprintw(22, 0, "Error: Servo init pose service not available");
      refresh();
      napms(2000);
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    mvprintw(22, 0, "Set init pose...");
    refresh();

    auto future = set_init_pose_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->success) {
        mvprintw(22, 0, "Init pose set successfully");
      }
      else {
        mvprintw(22, 0, "Error: Failed to set init pose");
      }
    }
    else {
      mvprintw(22, 0, "Error: Failed to set init pose (timeout)");
    }
    napms(2000);
  }

  void OffsetEditor::handle_input() {
    int ch;
    if ((ch = getch()) != 'q') {
      switch (ch) {
      case 'r': load_yaml(); draw_UI();  break;
      case 'p': ping_all_servos(); draw_UI(); break;
      case ' ': toggle_torque(); break;
      case 'i': set_init_pose(); break;
      case KEY_UP: if (selectedJoint_ > 0) selectedJoint_--; break;
      case KEY_DOWN: if (selectedJoint_ < (*categories_[selectedCategoryIndex_]).size() - 1) selectedJoint_++; break;
      case '	': selectedCategoryIndex_ = (selectedCategoryIndex_ + 1) % categories_.size(); selectedJoint_ = 0; break;
      case 's': save_yaml(); break;
      case KEY_LEFT:
        (*categories_[selectedCategoryIndex_])[selectedJoint_].offset -= 0.01;
        break;
      case KEY_RIGHT:
        (*categories_[selectedCategoryIndex_])[selectedJoint_].offset += 0.01;
        break;
      }
      draw_UI();
    }

  }

}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  initscr();
  start_color();
  init_pair(1, COLOR_GREEN, COLOR_BLACK);
  init_pair(2, COLOR_RED, COLOR_BLACK);
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  // Create the node
  // auto node = std::make_shared<rclcpp::Node>("offset_editor");
  // Create the editor and passing the node to it
  auto editor = std::make_shared<offset_editor::OffsetEditor>();


  while (rclcpp::ok()) {
    editor->draw_UI();
    editor->handle_input();

    try {
      rclcpp::spin_some(editor);
    }
    catch (const std::exception& e) {
      // so that we can ctrl+c and destroy the spin_some() properly.
    }
  }

  endwin();
  rclcpp::shutdown();

  return 0;
}
