#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>

struct Device {
  std::string name;
  int id;
  double joint_offset;
  std::string model;
};

struct robot {
  std::string port_name;
  int baudrate;
  double protocol;
  std::string default_joint;
  std::map<int, Device> joint_map;
};

// Config parseConfig(const std::string& file_path) {
//   Config config;
//   YAML::Node config_node = YAML::LoadFile(file_path);

//   config.port_name = config_node["PORTNAME"].as<std::string>();
//   config.baudrate = config_node["BAUDRATE"].as<int>();
//   config.protocol = config_node["PROTOCOL"].as<double>();
//   config.default_joint = config_node["DEFAULT_JOINT"].as<std::string>();

//   YAML::Node devices_node = config_node["DEVICE"];
//   for (YAML::const_iterator it = devices_node.begin(); it != devices_node.end(); ++it) {
//     int key = it->first.as<int>();
//     YAML::Node device_node = it->second;

//     Device device;
//     device.name = device_node["NAME"].as<std::string>();
//     device.id = device_node["ID"].as<int>();
//     device.model = device_node["MODEL"].as<std::string>();

//     config.devices[key] = device;
//   }

//   return config;
// }

int main(int argc, char** argv) {
  std::shared_ptr<robot> robot_;
  robot_ = std::make_shared<robot>();
  // find the config file path
  std::string t = ament_index_cpp::get_package_share_directory("experiment");
  std::string f = t + "/config/tes.yaml";
  YAML::Node config_node;

  try {
    config_node = YAML::LoadFile(f);
  } catch (const std::exception& e) {
    return false;
  }

  // parse portname, baudrate, protocol, and default joint

  robot_->port_name = config_node["PORTNAME"].as<std::string>();
  robot_->baudrate = config_node["BAUDRATE"].as<int>();
  robot_->protocol = config_node["PROTOCOL"].as<double>();
  robot_->default_joint = config_node["DEFAULT_JOINT"].as<std::string>();

  // parse every device joint_name, id, & info
  YAML::Node devices_node = config_node["DEVICE"];
  for (YAML::const_iterator it = devices_node.begin(); it != devices_node.end(); ++it) {
    int key = it->first.as<uint8_t>();
    YAML::Node device_node = it->second;

    Device device;
    device.name = device_node["NAME"].as<std::string>();
    device.id = device_node["ID"].as<uint8_t>();
    device.model = device_node["MODEL"].as<std::string>();

    robot_->joint_map[key] = device;
  }

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("experiment");
  std::string file_path = package_share_directory + "/config/offset.yaml";
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(file_path);
  } catch (const std::exception& e) {
    return false;
  }
  YAML::Node offset_node = doc["offset"];
  for (YAML::const_iterator it = offset_node.begin(); it != offset_node.end(); ++it) {
    int i = 0;
    for (auto& key : robot_->joint_map) {
      i++;
      // find the mathing pair of joint_name from offset to robot_->joint_map index key
      if (key.second.name == it->first.as<std::string>()) {
        robot_->joint_map[i].joint_offset = it->second.as<double>();
      }
    }
  }

  std::cout << "PORTNAME: " << robot_->port_name << std::endl;
  std::cout << "BAUDRATE: " << robot_->baudrate << std::endl;
  std::cout << "PROTOCOL: " << robot_->protocol << std::endl;
  std::cout << "DEFAULT_JOINT: " << robot_->default_joint << std::endl;

  for (const auto& device_pair : robot_->joint_map) {
    std::cout << "DEVICE " << device_pair.first << ":" << std::endl;
    std::cout << "  NAME: " << device_pair.second.name << std::endl;
    std::cout << "  ID: " << device_pair.second.id << std::endl;
    std::cout << "  JOINT_OFFSET: " << device_pair.second.joint_offset << std::endl;
    std::cout << "  MODEL: " << device_pair.second.model << std::endl;
  }
}
