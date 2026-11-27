#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

bool DEBUG_PRINT = true;

YAML::Node config;
YAML::Node joints;
std::unordered_map<std::string, int> dxls_index_;
std::unordered_map<int, std::vector<double>> stored_values;
std::unordered_map<int, std::vector<double>> joint_trajectories;

std::vector<double> minimum_jerk_trajectory(double init_pos, double target_pos, double total_time = 0.5, double dt = 0.01) {
  std::vector<double> trajectory;
  double d = total_time;

  // Calculate the trajectory for each time step
  for (double t = 0; t < d; t += dt) {
    double normalized_time = t / d;
    double joint_position = init_pos + (target_pos - init_pos) *
      (10 * std::pow(normalized_time, 3) - 15 * std::pow(normalized_time, 4) + 6 * std::pow(normalized_time, 5));
    trajectory.push_back(joint_position);
  }

  return trajectory;
}

int main(int argc, char** argv) {
  try {
    dxls_index_ = {
          {"head_pan", 0}, {"head_tilt", 1}, {"l_el", 2}, {"l_sho_roll", 3},
          {"l_sho_pitch", 4}, {"r_el", 5}, {"r_sho_roll", 6}, {"r_sho_pitch", 7},
          {"l_ank_roll", 8}, {"l_ank_pitch", 9}, {"l_knee", 10}, {"l_hip_pitch", 11},
          {"l_hip_roll", 12}, {"l_hip_yaw", 13}, {"r_ank_roll", 14}, {"r_ank_pitch", 15},
          {"r_knee", 16}, {"r_hip_pitch", 17}, {"r_hip_roll", 18}, {"r_hip_yaw", 19}
    };

    std::string a = ament_index_cpp::get_package_share_directory("yantra_animation");
    std::string data_play = a + "/data/hello.yaml";


    try {
      // Load the YAML file
      config = YAML::LoadFile(data_play);

      if (config["joints"]) {
        joints = config["joints"];


        // Iterate through each joint
        for (YAML::const_iterator it = joints.begin(); it != joints.end(); ++it) {
          std::string joint_name = it->first.as<std::string>();

          // Get the joint values
          std::vector<double> values = it->second.as<std::vector<double>>();

          // Send each value to the corresponding command interface
          if (dxls_index_.find(joint_name) != dxls_index_.end()) {
            int id = dxls_index_[joint_name];
            stored_values[id] = values;
          }
          else {
            std::cerr << "Warning: Joint name '" << joint_name << "' not found in dxls_index_ map." << std::endl;
          }
        }
      }
      else {
        std::cerr << "The 'joints' key is missing in the YAML file!" << std::endl;
      }
    }
    catch (const YAML::Exception& e) {
      std::cerr << "Error loading or parsing YAML file: " << e.what() << std::endl;
    }

    for (const auto& pair : stored_values) {
      int id = pair.first;
      const std::vector<double>& values = pair.second;

      for (int i = 0; i < values.size() - 1; ++i) {
        int value = values[i];
        int value_next = values[i + 1];
        std::vector<double> new_trajectory = minimum_jerk_trajectory(value, value_next);
        joint_trajectories[id].insert(
          joint_trajectories[id].end(), new_trajectory.begin(), new_trajectory.end()
        );
      }
    }

    // Print the updated content
    for (const auto& [key, trajectory] : joint_trajectories) {
      std::cout << "Joint " << key << ": ";
      for (double value : trajectory) {
        std::cout << value << " ";
        // command_interfaces_[key].set_value(value);
      }
      std::cout << "\n";
    }


    std::cout << "on_configure" << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
}

