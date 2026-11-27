#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "yantra_msgs/msg/game_state.hpp"

#include "game_controller/game_controller_client_t.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(2.0);  // 2 Hz frequency
  // Create an instance of the GameController node
  auto game_controller_node = std::make_shared<rclcpp::Node>("game_controller_node");

  auto data_publisher = game_controller_node->create_publisher<yantra_msgs::msg::GameState>("yantra/game_state", 10);

  auto team_id = game_controller_node->declare_parameter<int>("team_id", 1);
  auto robot_id = game_controller_node->declare_parameter<int>("robot_id", 1);
  RCLCPP_INFO(game_controller_node->get_logger(), "Team ID: %d, Robot ID: %d", team_id, robot_id);
  
  robocup::game_controller_client_t client(team_id, robot_id);


  while (rclcpp::ok()) {
    client.update();
    if (!client.is_server_responds()) {

      const auto& data = client.get_game_ctrl_data();
      yantra_msgs::msg::GameState game_data;

      game_data.game_state = (int)data.state;
      game_data.secondary_state = (int)data.secondary_state;
      game_data.first_half = (int)data.first_half;
      game_data.seconds_remaining = (int)data.secs_remaining;
      game_data.secondary_seconds_remaining = (int)data.secondary_time;

      if ((int)data.secondary_time == 0 && (int)data.state == STATE_PLAYING)
        game_data.has_kick_off = true;
      else
        game_data.has_kick_off = false;

      team_info_t team_data;
      team_info_t rival_data;

      for (int i = 0; i < 2; i++)
      {
        if ((int)data.teams[i].team_number == team_id)
        {
          team_data = data.teams[i];
          break;
        }
        else
        {
          rival_data = data.teams[i];
        }
      }

      game_data.single_shots = (int)team_data.single_shots;
      game_data.penalty_shot = (int)team_data.penalty_shot;

      game_data.own_score = (int)team_data.score;
      game_data.rival_score = (int)rival_data.score;

      robot_info_t robot_data = team_data.players[robot_id - 1];
      game_data.penalty = (int)robot_data.penalty;
      game_data.penalized = (int)robot_data.penalty == 0 ? 0 : 1;

      if ((int)data.state == STATE_READY)
        game_data.allowed_to_move = true;
      else if ((int)data.kick_off_team == (int)team_data.team_colour && (int)data.state == STATE_PLAYING && (int)robot_data.penalty == 0)
        game_data.allowed_to_move = true;
      else if ((int)data.kick_off_team != (int)team_data.team_colour && (int)data.state == STATE_PLAYING
        && game_data.has_kick_off && (int)robot_data.penalty == 0)
        game_data.allowed_to_move = true;
      else
        game_data.allowed_to_move = false;

      game_data.seconds_till_unpenalized = (int)robot_data.secs_till_unpenalised;

      data_publisher->publish(game_data);

    }
    else {
      RCLCPP_WARN(game_controller_node->get_logger(), "Waiting for Game Controller messages");
    }

    // Spin the node to process callbacks
    rclcpp::spin_some(game_controller_node);
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
