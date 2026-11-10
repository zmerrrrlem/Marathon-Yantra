#ifndef QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_VISUALIZER_H_
#define QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_VISUALIZER_H_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <quintic_walk/walk_engine.hpp>
#include <quintic_walk/walk_utils.hpp>
#include <ranges>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "altair_msgs/msg/walk_debug.hpp"
#include "altair_msgs/msg/walk_engine_debug.hpp"
#include "quintic_splines/abstract_ik.hpp"
#include "quintic_splines/abstract_visualizer.hpp"

namespace quintic_walk {
  class WalkVisualizer : public quintic_splines::AbstractVisualizer {
  public:
  explicit WalkVisualizer(rclcpp::Node::SharedPtr node, walking::Params::Node::Tf tf_config);

    visualization_msgs::msg::Marker createArrowMarker(const std::string& name_space, const std::string& frame,
      const geometry_msgs::msg::Pose& pose,
      const std_msgs::msg::ColorRGBA& color);

    std::pair<altair_msgs::msg::WalkEngineDebug, visualization_msgs::msg::MarkerArray> buildEngineDebugMsgs(
      WalkResponse response, WalkResponse stabilized_response);
    std::pair<altair_msgs::msg::WalkDebug, visualization_msgs::msg::MarkerArray> buildIKDebugMsgs(
      WalkResponse response, const geometry_msgs::msg::Pose& left_foot_pose, const geometry_msgs::msg::Pose& right_foot_pose);
    visualization_msgs::msg::MarkerArray buildWalkMarkers(WalkResponse response, WalkResponse stabilized_response );
    void publishDebug(const WalkResponse& current_response, const WalkResponse& current_stabilized_response,
      const geometry_msgs::msg::Pose& left_foot_pose, const geometry_msgs::msg::Pose& right_foot_pose,
      const bool& rviz_visualization);

    std_msgs::msg::ColorRGBA colorFactory(double r, double g, double b) {
      std_msgs::msg::ColorRGBA color;
      color.r = r;
      color.g = g;
      color.b = b;
      color.a = 1.0;
      return color;
    }

    const std_msgs::msg::ColorRGBA BLACK = colorFactory(0.0, 0.0, 0.0);
    const std_msgs::msg::ColorRGBA BLUE = colorFactory(0.0, 0.0, 1.0);
    const std_msgs::msg::ColorRGBA GREEN = colorFactory(0.0, 1.0, 0.0);
    const std_msgs::msg::ColorRGBA ORANGE = colorFactory(1.0, 0.5, 0.0);
    const std_msgs::msg::ColorRGBA RED = colorFactory(1.0, 0.0, 0.0);
    const std_msgs::msg::ColorRGBA WHITE = colorFactory(1.0, 1.0, 1.0);
    const std_msgs::msg::ColorRGBA YELLOW = colorFactory(1.0, 1.0, 0.0);

  private:
  rclcpp::Node::SharedPtr node_;

    walking::Params::Node::Tf tf_config_;

    rclcpp::Publisher<altair_msgs::msg::WalkDebug>::SharedPtr pub_debug_;
    rclcpp::Publisher<altair_msgs::msg::WalkEngineDebug>::SharedPtr pub_engine_debug_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
    visualization_msgs::msg::Marker trunk_trail_marker_;
    visualization_msgs::msg::Marker trunk_stabilized_trail_marker_;
    tf2_ros::Buffer tf_buffer_; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; 
  };
}  // namespace quintic_walk

#endif  // QUINTIC_WALK_INCLUDE_QUINTIC_WALK_WALK_VISUALIZER_H_