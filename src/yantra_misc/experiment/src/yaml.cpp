#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

struct Device {
  std::string name;
  int id;
  double joint_offset;
  std::string model;
};

struct Servo {
  std::string name;
  int id;
  double joint_offset;
  std::string model;
};

struct Robot {
  std::string port_name;
  int baudrate;
  double protocol;
  std::string default_joint;
  std::map<int, Servo> joint_map;
};

bool DEBUG_PRINT = true;
std::shared_ptr<Robot> robot_;

int main(int argc, char** argv) {
  std::string a = ament_index_cpp::get_package_share_directory("dynamixel_controller");
  std::string dxl_config = a + "/config/dxl_init.yaml";
  int i = 0;
  YAML::Node config_node;
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(dxl_config);
    // iterate joint name in dxl_init.yaml
    for (YAML::const_iterator it_doc = doc.begin(); it_doc != doc.end(); it_doc++) {
      std::string joint_name = it_doc->first.as<std::string>();

      YAML::Node joint_node = doc[joint_name];

      // interate child component of joint_node
      for (YAML::const_iterator it_joint = joint_node.begin(); it_joint != joint_node.end(); it_joint++) {
        // get the item_name and value of joint_node
        std::string item_name = it_joint->first.as<std::string>();
        uint32_t value = it_joint->second.as<uint32_t>();
        std::cout << "Item name: " << item_name << std::endl;
        i = 0;
        for (auto& key : robot_->joint_map) {
          i++;

          // find the mathing pair of joint_name from offset.yaml to its id
          if (key.second.name == joint_name) {
            if (DEBUG_PRINT)
              printf("ID: %s JOINTNAME: %s ITEM_NAME: %s VALUE: %s", i, joint_name, item_name, value);
            // write the value to the register
            break;
          }
        }
      }
    }
  } catch (const std::exception& e) {
    printf("Dynamixel Init file not found.");
  }
}
