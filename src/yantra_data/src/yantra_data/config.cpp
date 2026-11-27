#include "yantra_data/config.hpp"



yantra::Config::Config() {
    
    this->core_config   = YAML::LoadFile("/yantra-os/src/yantra_data/config/core_config.yaml");
    this->robot_config  = YAML::LoadFile("/yantra-os/src/yantra_data/config/robot_config.yaml");
    this->joint_config  = YAML::LoadFile("/yantra-os/src/yantra_data/config/joint_config.yaml");

    this->id    = this->robot_config["id"].as<std::string>();
    this->ip    = this->robot_config["ip"].as<std::string>();
}