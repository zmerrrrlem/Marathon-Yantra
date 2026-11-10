#include <cstdlib>
#include <string>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "quintic_walk/walk_engine.hpp"
#include "quintic_walk/walk_utils.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("walk_engine_trajectory");

  const std::string csv_path = (argc > 1) ? argv[1] : "walk_engine_trajectory.csv";
  const int num_steps = (argc > 2) ? std::atoi(argv[2]) : 8;
  const int samples_per_step = (argc > 3) ? std::atoi(argv[3]) : 100;

  std::string config_dir = ament_index_cpp::get_package_share_directory("quintic_walk") + "/test/";
  std::string config_file = config_dir + "config.yaml";
  YAML::Node conf;
  conf = YAML::LoadFile(config_file.c_str());
  auto conf_engine = conf["engine"];

  walking::Params::Engine config{};
  config.freq = conf_engine["freq"].as<double>();
  config.double_support_ratio = conf_engine["double_support_ratio"].as<double>();
  config.first_step_swing_factor = conf_engine["first_step_swing_factor"].as<double>();
  config.first_step_trunk_phase = conf_engine["first_step_trunk_phase"].as<double>();
  config.foot_distance = conf_engine["foot_distance"].as<double>();
  config.foot_rise = conf_engine["foot_rise"].as<double>();
  config.trunk_swing = conf_engine["trunk_swing"].as<double>();
  config.trunk_height = conf_engine["trunk_height"].as<double>();
  config.trunk_pitch = conf_engine["trunk_pitch"].as<double>();
  config.trunk_pitch_p_coef_forward = conf_engine["trunk_pitch_p_coef_forward"].as<double>();
  config.trunk_pitch_p_coef_turn = conf_engine["trunk_pitch_p_coef_turn"].as<double>();
  config.trunk_phase = conf_engine["trunk_phase"].as<double>();
  config.foot_z_pause = conf_engine["foot_z_pause"].as<double>();
  config.foot_put_down_z_offset = conf_engine["foot_put_down_z_offset"].as<double>();
  config.foot_put_down_phase = conf_engine["foot_put_down_phase"].as<double>();
  config.foot_apex_phase = conf_engine["foot_apex_phase"].as<double>();
  config.foot_apex_pitch = conf_engine["foot_apex_pitch"].as<double>();
  config.foot_overshoot_ratio = conf_engine["foot_overshoot_ratio"].as<double>();
  config.foot_overshoot_phase = conf_engine["foot_overshoot_phase"].as<double>();
  config.trunk_x_offset = conf_engine["trunk_x_offset"].as<double>();
  config.trunk_y_offset = conf_engine["trunk_y_offset"].as<double>();
  config.trunk_z_movement = conf_engine["trunk_z_movement"].as<double>();
  config.trunk_pause = conf_engine["trunk_pause"].as<double>();
  config.trunk_x_offset_p_coef_forward = conf_engine["trunk_x_offset_p_coef_forward"].as<double>();
  config.trunk_x_offset_p_coef_turn = conf_engine["trunk_x_offset_p_coef_turn"].as<double>();
  config.kick_length = conf_engine["kick_length"].as<double>();
  config.kick_vel = conf_engine["kick_vel"].as<double>();
  config.kick_phase = conf_engine["kick_phase"].as<double>();
  config.kick_put_down_phase = conf_engine["kick_put_down_phase"].as<double>();
  config.kick_rise_factor = conf_engine["kick_rise_factor"].as<double>();

  quintic_walk::WalkEngine engine(node, config);

  quintic_walk::WalkRequest request;
  request.linear_orders = { 0.04, 0.0, 0.0 };
  request.angular_z = 0.0;
  request.stop_walk = false;
  request.walkable_state = true;
  engine.setGoals(request);

  engine.plotFullGaitTrajectory(csv_path, num_steps, samples_per_step);
  RCLCPP_INFO(node->get_logger(), "Trajectory written to %s (steps=%d, samples/step=%d)", csv_path.c_str(), num_steps,
              samples_per_step);

  rclcpp::shutdown();
  return 0;
}
