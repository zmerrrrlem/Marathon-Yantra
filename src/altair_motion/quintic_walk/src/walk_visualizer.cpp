#include "quintic_walk/walk_visualizer.hpp"

namespace quintic_walk {
  WalkVisualizer::WalkVisualizer(rclcpp::Node::SharedPtr node, walking::Params::Node::Tf tf_config)
    : node_(node),
    tf_config_(tf_config),
    pub_debug_(node_->create_publisher<altair_msgs::msg::WalkDebug>("walk_debug", 1)),
    pub_engine_debug_(node_->create_publisher<altair_msgs::msg::WalkEngineDebug>("walk_engine_debug", 1)),
    pub_debug_marker_(node_->create_publisher<visualization_msgs::msg::MarkerArray>("walk_debug_marker", 1)),
    tf_buffer_(node_->get_clock()),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
    trunk_trail_marker_.header.frame_id = tf_config_.base_link_frame;
    trunk_trail_marker_.type = trunk_trail_marker_.LINE_STRIP;
    trunk_trail_marker_.action = trunk_trail_marker_.ADD;
    trunk_trail_marker_.ns = "trunk_trail";
    trunk_trail_marker_.id = 4;
    trunk_trail_marker_.scale.x = 0.001;
    trunk_trail_marker_.color.r = 1.0;
    trunk_trail_marker_.color.g = 0.0;
    trunk_trail_marker_.color.b = 0.0;
    trunk_trail_marker_.color.a = 0.5;
    trunk_trail_marker_.lifetime = rclcpp::Duration::from_nanoseconds(0.0);

    trunk_stabilized_trail_marker_.header.frame_id = tf_config_.base_link_frame;
    trunk_stabilized_trail_marker_.type = trunk_stabilized_trail_marker_.LINE_STRIP;
    trunk_stabilized_trail_marker_.action = trunk_stabilized_trail_marker_.ADD;
    trunk_stabilized_trail_marker_.ns = "trunk_stabilized_trail";
    trunk_stabilized_trail_marker_.id = 4;
    trunk_stabilized_trail_marker_.scale.x = 0.001;
    trunk_stabilized_trail_marker_.color.r = 0.0;
    trunk_stabilized_trail_marker_.color.g = 1.0;
    trunk_stabilized_trail_marker_.color.b = 0.0;
    trunk_stabilized_trail_marker_.color.a = 1.0;
    trunk_stabilized_trail_marker_.lifetime = rclcpp::Duration::from_nanoseconds(0.0);

  }
  void WalkVisualizer::publishDebug(const WalkResponse& current_response, const WalkResponse& current_stabilized_response,
    const geometry_msgs::msg::Pose& left_foot_pose, const geometry_msgs::msg::Pose& right_foot_pose,
    const bool& rviz_visualization) {
    visualization_msgs::msg::MarkerArray marker_array;

    auto [engine_debug, engine_markers] = buildEngineDebugMsgs(current_response, current_stabilized_response);
    pub_engine_debug_->publish(engine_debug);

    auto [ik_debug, ik_markers] = buildIKDebugMsgs(current_stabilized_response, left_foot_pose, right_foot_pose);
    pub_debug_->publish(ik_debug);

    if (rviz_visualization) {
      marker_array.markers.insert(marker_array.markers.end(), engine_markers.markers.begin(), engine_markers.markers.end());
      marker_array.markers.insert(marker_array.markers.end(), ik_markers.markers.begin(), ik_markers.markers.end());

      auto walk_markers = buildWalkMarkers(current_response, current_stabilized_response);
      marker_array.markers.insert(marker_array.markers.end(), walk_markers.markers.begin(), walk_markers.markers.end());
    }


    pub_debug_marker_->publish(marker_array);
  }

  std::pair<altair_msgs::msg::WalkEngineDebug, visualization_msgs::msg::MarkerArray>
    WalkVisualizer::buildEngineDebugMsgs(WalkResponse response, WalkResponse stabilized_response) {
    // Here we will convert the walk engine response to a various debug messages and RViz markers
    // Initialize output containers
    altair_msgs::msg::WalkEngineDebug msg;
    visualization_msgs::msg::MarkerArray marker_array;

    // Copy some data into the debug message
    msg.is_left_support = response.is_left_support_foot;
    msg.is_double_support = response.is_double_support;
    msg.header.stamp = node_->now();
    msg.phase_time = response.phase;
    msg.traj_time = response.traj_time;
    // Copy walk engine state
    static const std::unordered_map<WalkState, std::string> state_string_mapping = {
        {WalkState::IDLE, "idle"},
        {WalkState::START_MOVEMENT, "start_movement"},
        {WalkState::START_STEP, "start_step"},
        {WalkState::WALKING, "walking"},
        {WalkState::PAUSED, "paused"},
        {WalkState::KICK, "kick"},
        {WalkState::STOP_STEP, "stop_step"},
        {WalkState::STOP_MOVEMENT, "stop_movement"} };
    msg.state.data = state_string_mapping.at(response.state);
    msg.state_number = static_cast<int>(response.state);

    // Define current support foot frame
    std::string current_support_frame;
    if (response.is_left_support_foot) {
      current_support_frame = tf_config_.l_sole_frame;
    }
    else {
      current_support_frame = tf_config_.r_sole_frame;
    }

    // Create placeholder floats
    double _1, _2;
    // Copy transform of the last footstep position (and orientation) to the debug message
    msg.footstep_last.x = response.support_to_last.getOrigin()[0];
    msg.footstep_last.y = response.support_to_last.getOrigin()[1];
    tf2::Matrix3x3(response.support_to_last.getRotation()).getRPY(_1, _2, msg.footstep_last.z);

    // Copy transform of the next footstep position (and orientation) to the debug message
    msg.footstep_next.x = response.support_to_next.getOrigin()[0];
    msg.footstep_next.y = response.support_to_next.getOrigin()[1];
    tf2::Matrix3x3(response.support_to_next.getRotation()).getRPY(_1, _2, msg.footstep_next.z);

    // Copy cartesian coordinates of the currently flying foot relative to the support foot to the debug message
    tf2::toMsg(response.support_foot_to_flying_foot, msg.fly_goal);
    // Create an additional marker for the flying foot goal
    marker_array.markers.push_back(createArrowMarker("engine_fly_goal", current_support_frame, msg.fly_goal, BLUE));
    RCLCPP_INFO_ONCE(node_->get_logger(),
      "Color for the Engine Debug marker, showing where the flying foot and trunk should be, is blue!");

    // Copy the rotation of the flying foot relative to the support foot to the debug message
    tf2::Matrix3x3(response.support_foot_to_flying_foot.getRotation())
      .getRPY(msg.fly_euler.x, msg.fly_euler.y, msg.fly_euler.z);

    // Copy cartesian coordinates of the trunk goal relative to the support foot to the debug message
    tf2::toMsg(response.support_foot_to_trunk, msg.trunk_goal);
    tf2::toMsg(stabilized_response.support_foot_to_trunk, msg.stabilized_trunk_goal);
    // Create an additional marker for the trunk goal
    marker_array.markers.push_back(createArrowMarker("engine_trunk_goal", current_support_frame, msg.trunk_goal, BLUE));

    // TODO check this!!!
    msg.trunk_goal_abs = msg.trunk_goal;
    if (msg.trunk_goal_abs.position.y > 0) {
      msg.trunk_goal_abs.position.y -= response.foot_distance / 2;
    }
    else {
      msg.trunk_goal_abs.position.y += response.foot_distance / 2;
    }

    tf2::Matrix3x3(response.support_foot_to_flying_foot.getRotation())
      .getRPY(msg.trunk_euler.x, msg.trunk_euler.y, msg.trunk_euler.z);

    // resulting trunk pose
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    marker_array.markers.push_back(createArrowMarker("trunk_result", tf_config_.base_link_frame, pose, GREEN));
    RCLCPP_INFO_ONCE(node_->get_logger(), "Color for the Engine Debug marker, showing where the trunk is, is green!");

    return { msg, marker_array };
  }

  std::pair<altair_msgs::msg::WalkDebug, visualization_msgs::msg::MarkerArray> WalkVisualizer::buildIKDebugMsgs(
    WalkResponse response, const geometry_msgs::msg::Pose& left_foot_pose, const geometry_msgs::msg::Pose& right_foot_pose) {
    altair_msgs::msg::WalkDebug msg;
    visualization_msgs::msg::MarkerArray marker_array;

    tf2::Transform trunk_to_support_foot = response.support_foot_to_trunk.inverse();
    tf2::Transform trunk_to_flying_foot = trunk_to_support_foot * response.support_foot_to_flying_foot;

    // Copy goals into the message
    tf2::toMsg(trunk_to_support_foot, msg.support_foot_goal);
    tf2::toMsg(trunk_to_flying_foot, msg.fly_foot_goal);
    // Set left and right foot goals based on the support foot
    if (response.is_left_support_foot) {
      msg.left_foot_goal = msg.support_foot_goal;
      msg.right_foot_goal = msg.fly_foot_goal;
    }
    else {
      msg.left_foot_goal = msg.fly_foot_goal;
      msg.right_foot_goal = msg.support_foot_goal;
    }
    // Create additional markers for the foot goals
    marker_array.markers.push_back(
      createArrowMarker("engine_left_goal", tf_config_.base_link_frame, msg.left_foot_goal, ORANGE));
    marker_array.markers.push_back(
      createArrowMarker("engine_right_goal", tf_config_.base_link_frame, msg.right_foot_goal, ORANGE));
    RCLCPP_INFO_ONCE(node_->get_logger(), "Color for the IK marker, showing where the feet should be, is orange!");

    // IK results are directly available from the analytic solver outputs
    msg.left_foot_ik_result = left_foot_pose;
    msg.right_foot_ik_result = right_foot_pose;
    if (response.is_left_support_foot) {
      msg.support_foot_ik_result = msg.left_foot_ik_result;
      msg.fly_foot_ik_result = msg.right_foot_ik_result;
    }
    else {
      msg.support_foot_ik_result = msg.right_foot_ik_result;
      msg.fly_foot_ik_result = msg.left_foot_ik_result;
    }
    marker_array.markers.push_back(
      createArrowMarker("ik_left", tf_config_.base_link_frame, msg.left_foot_ik_result, GREEN));
    marker_array.markers.push_back(
      createArrowMarker("ik_right", tf_config_.base_link_frame, msg.right_foot_ik_result, GREEN));
    RCLCPP_INFO_ONCE(node_->get_logger(), "Color for the IK marker, showing the ik result, is green!");

    // IK offsets
    tf2::Vector3 support_off;
    tf2::Vector3 fly_off;
    tf2::Vector3 tf_vec_left;
    tf2::Vector3 tf_vec_right;
    tf2::Transform left_tf;
    tf2::Transform right_tf;
    tf2::fromMsg(left_foot_pose, left_tf);
    tf2::fromMsg(right_foot_pose, right_tf);
    tf_vec_left = left_tf.getOrigin();
    tf_vec_right = right_tf.getOrigin();
    geometry_msgs::msg::Vector3 vect_msg;
    if (response.is_left_support_foot) {
      support_off = trunk_to_support_foot.getOrigin() - tf_vec_left;
      fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_right;
      vect_msg.x = support_off.x();
      vect_msg.y = support_off.y();
      vect_msg.z = support_off.z();
      msg.left_foot_ik_offset = vect_msg;
      vect_msg.x = fly_off.x();
      vect_msg.y = fly_off.y();
      vect_msg.z = fly_off.z();
      msg.right_foot_ik_offset = vect_msg;
    }
    else {
      support_off = trunk_to_support_foot.getOrigin() - tf_vec_right;
      fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_left;
      vect_msg.x = fly_off.x();
      vect_msg.y = fly_off.y();
      vect_msg.z = fly_off.z();
      msg.left_foot_ik_offset = vect_msg;
      vect_msg.x = support_off.x();
      vect_msg.y = support_off.y();
      vect_msg.z = support_off.z();
      msg.right_foot_ik_offset = vect_msg;
    }
    vect_msg.x = support_off.x();
    vect_msg.y = support_off.y();
    vect_msg.z = support_off.z();
    msg.support_foot_ik_offset = vect_msg;
    vect_msg.x = fly_off.x();
    vect_msg.y = fly_off.y();
    vect_msg.z = fly_off.z();
    msg.fly_foot_ik_offset = vect_msg;

    // Actual foot positions determined by the IK solver
    msg.left_foot_position = left_foot_pose;
    msg.right_foot_position = right_foot_pose;
    if (response.is_left_support_foot) {
      msg.support_foot_position = msg.left_foot_position;
      msg.fly_foot_position = msg.right_foot_position;
    }
    else {
      msg.support_foot_position = msg.right_foot_position;
      msg.fly_foot_position = msg.left_foot_position;
    }

    // Calculate offsets between the actual foot positions and the goals (meaning the IK solver error)
    tf_vec_left = left_tf.getOrigin();
    tf_vec_right = right_tf.getOrigin();
    if (response.is_left_support_foot) {
      support_off = trunk_to_support_foot.getOrigin() - tf_vec_left;
      fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_right;
      vect_msg.x = support_off.x();
      vect_msg.y = support_off.y();
      vect_msg.z = support_off.z();
      msg.left_foot_actual_offset = vect_msg;
      vect_msg.x = fly_off.x();
      vect_msg.y = fly_off.y();
      vect_msg.z = fly_off.z();
      msg.right_foot_actual_offset = vect_msg;
    }
    else {
      support_off = trunk_to_support_foot.getOrigin() - tf_vec_right;
      fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_left;
      vect_msg.x = fly_off.x();
      vect_msg.y = fly_off.y();
      vect_msg.z = fly_off.z();
      msg.left_foot_actual_offset = vect_msg;
      vect_msg.x = support_off.x();
      vect_msg.y = support_off.y();
      vect_msg.z = support_off.z();
      msg.right_foot_actual_offset = vect_msg;
    }
    vect_msg.x = support_off.x();
    vect_msg.y = support_off.y();
    vect_msg.z = support_off.z();
    msg.support_foot_actual_offset = vect_msg;
    vect_msg.x = fly_off.x();
    vect_msg.y = fly_off.y();
    vect_msg.z = fly_off.z();
    msg.fly_foot_actual_offset = vect_msg;

    return { msg, marker_array };
  }

  visualization_msgs::msg::Marker WalkVisualizer::createArrowMarker(const std::string& name_space,
    const std::string& frame,
    const geometry_msgs::msg::Pose& pose,
    const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = frame;

    marker_msg.type = marker_msg.ARROW;
    marker_msg.ns = name_space;
    marker_msg.action = marker_msg.ADD;
    marker_msg.pose = pose;
    marker_msg.color = color;

    geometry_msgs::msg::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.003;
    scale.z = 0.003;
    marker_msg.scale = scale;
    marker_msg.id = 0;

    return marker_msg;
  }

  visualization_msgs::msg::MarkerArray WalkVisualizer::buildWalkMarkers(WalkResponse response, WalkResponse stabilized_response) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Create a marker for the last step
    visualization_msgs::msg::Marker support_foot_marker_msg;
    if (response.is_left_support_foot) {
      support_foot_marker_msg.header.frame_id = tf_config_.l_sole_frame;
    }
    else {
      support_foot_marker_msg.header.frame_id = tf_config_.r_sole_frame;
    }
    support_foot_marker_msg.type = support_foot_marker_msg.CUBE;
    support_foot_marker_msg.action = 0;
    support_foot_marker_msg.lifetime = rclcpp::Duration::from_nanoseconds(0.0);
    support_foot_marker_msg.scale.x = 0.2;
    support_foot_marker_msg.scale.y = 0.1;
    support_foot_marker_msg.scale.z = 0.01;
    support_foot_marker_msg.ns = "last_step";
    support_foot_marker_msg.id = 1;
    support_foot_marker_msg.color = BLACK;
    support_foot_marker_msg.color.a = 0.5;
    tf2::toMsg(response.support_to_last, support_foot_marker_msg.pose);
    marker_array.markers.push_back(support_foot_marker_msg);

    // This step center
    auto step_center_marker_msg(support_foot_marker_msg);
    step_center_marker_msg.ns = "step_center";
    step_center_marker_msg.id = 2;
    step_center_marker_msg.scale.x = 0.01;
    step_center_marker_msg.scale.y = 0.01;
    step_center_marker_msg.scale.z = 0.01;
    marker_array.markers.push_back(step_center_marker_msg);

    // Next step
    auto next_step_marker_msg(support_foot_marker_msg);
    next_step_marker_msg.id = 3;
    next_step_marker_msg.ns = "next_step";
    next_step_marker_msg.scale.x = 0.20;
    next_step_marker_msg.scale.y = 0.10;
    next_step_marker_msg.scale.z = 0.01;
    next_step_marker_msg.color = WHITE;
    next_step_marker_msg.color.a = 0.5;
    tf2::toMsg(response.support_to_next, next_step_marker_msg.pose);
    marker_array.markers.push_back(next_step_marker_msg);

    // FOR UNSTABILIZED RESPONSE
    geometry_msgs::msg::Point trunk_position;
    try {
      geometry_msgs::msg::TransformStamped transform_to_odom =
        tf_buffer_.lookupTransform("odom", tf_config_.base_link_frame, tf2::TimePointZero);
      tf2::Vector3 trunk_position_in_robot_frame(
        response.support_foot_to_trunk.getOrigin().x(),
        response.support_foot_to_trunk.getOrigin().y(),
        0.0);

      tf2::Transform transform;
      tf2::fromMsg(transform_to_odom.transform, transform);
      tf2::Vector3 trunk_position_in_odom_frame = transform * trunk_position_in_robot_frame;

      trunk_position.x = trunk_position_in_odom_frame.x();
      trunk_position.y = trunk_position_in_odom_frame.y();
      trunk_position.z = trunk_position_in_odom_frame.z();
    }
    catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform trunk position to odom frame: %s", ex.what());
      return marker_array;
    }
    trunk_trail_marker_.header.frame_id = "odom";  // Ensure the trail is in the odom frame
    trunk_trail_marker_.points.push_back(trunk_position);
    if (trunk_trail_marker_.points.size() > 5000) {
      trunk_trail_marker_.points.erase(trunk_trail_marker_.points.begin());
    }
    marker_array.markers.push_back(trunk_trail_marker_);

    // FOR STABILIZED RESPONSE
    geometry_msgs::msg::Point trunk_stabilized_position;
    try {
      geometry_msgs::msg::TransformStamped transform_to_odom =
        tf_buffer_.lookupTransform("odom", tf_config_.base_link_frame, tf2::TimePointZero);

      tf2::Vector3 trunk_position_in_robot_frame(
        stabilized_response.support_foot_to_trunk.getOrigin().x(),
        stabilized_response.support_foot_to_trunk.getOrigin().y(),
        0.0);

      // Apply the transform to get the trunk position in the odom frame
      tf2::Transform transform;
      tf2::fromMsg(transform_to_odom.transform, transform);
      tf2::Vector3 trunk_position_in_odom_frame = transform * trunk_position_in_robot_frame;

      // Set the transformed position
      trunk_stabilized_position.x = trunk_position_in_odom_frame.x();
      trunk_stabilized_position.y = trunk_position_in_odom_frame.y();
      trunk_stabilized_position.z = trunk_position_in_odom_frame.z();
    }
    catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "Could not transform trunk position to odom frame: %s", ex.what());
      return marker_array;
    }

    trunk_stabilized_trail_marker_.header.frame_id = "odom";
    trunk_stabilized_trail_marker_.points.push_back(trunk_stabilized_position);

    if (trunk_stabilized_trail_marker_.points.size() > 5000) {
      trunk_stabilized_trail_marker_.points.erase(trunk_stabilized_trail_marker_.points.begin());
    }
    marker_array.markers.push_back(trunk_stabilized_trail_marker_);
    return marker_array;
  }

}  // namespace quintic_walk