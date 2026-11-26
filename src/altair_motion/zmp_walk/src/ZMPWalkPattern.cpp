#include "zmp_walk/ZMPWalkPattern.h"
#include <iostream>

namespace zmp_walk {

  WalkPattern3DGenerator::WalkPattern3DGenerator(
    double sx, double sy, int N_step,
    double T_step, double dT, double zc,
    double q, double r, double g,
    int N, double step_angle,
    const std::vector<std::tuple<double, double, double>>& step_directions)
    : sx(sx), sy(sy), N_step(N_step), T_step(T_step), dT(dT),
    zc(zc), q(q), r(r), g(g), N(N), step_angle(step_angle),
    step_directions(step_directions) {

    // Walking phase parameters
    double_support_ratio = 0.2;  // 20% of step time in double support phase
    swing_ratio = 0.8;          // 80% of step time in swing phase

    // Foot placement constraints (in meters)
    min_foot_distance_x = 0.10;  // Minimum horizontal distance between feet
    max_foot_distance_x = 0.30;  // Maximum horizontal distance between feet
    min_foot_distance_y = 0.05;  // Minimum lateral distance between feet
    max_foot_distance_y = 0.20;  // Maximum lateral distance between feet

    // Adjust step parameters to satisfy constraints
    this->sy = std::max(this->sy, min_foot_distance_y);
    this->sy = std::min(this->sy, max_foot_distance_y);

    // Foot dimensions for visualization
    foot_length = 0.1;
    foot_width = 0.05;

    // Initialize COM state
    com_position = Eigen::Vector3d::Zero();
    com_velocity = Eigen::Vector3d::Zero();
    com_acceleration = Eigen::Vector3d::Zero();
  }

  std::vector<Footstep> WalkPattern3DGenerator::generate_footsteps(double sx, double sy, int N_step) {
    std::vector<Footstep> footsteps;

    // Initial positions with proper foot distance
    double foot_separation = std::max(min_foot_distance_y, std::min(sy, max_foot_distance_y));
    Eigen::Vector2d right_foot_pos(0, -foot_separation / 2);
    Eigen::Vector2d left_foot_pos(0, foot_separation / 2);
    double current_angle = 0;

    // Add initial stance
    Footstep right_initial;
    right_initial.position = right_foot_pos;
    right_initial.orientation = current_angle;
    right_initial.foot = "right";
    footsteps.push_back(right_initial);

    Footstep left_initial;
    left_initial.position = left_foot_pos;
    left_initial.orientation = current_angle;
    left_initial.foot = "left";
    footsteps.push_back(left_initial);

    // Start with left foot for first step
    bool is_left = true;

    // Common reference point for the robot's center (between feet)
    double center_x = 0;
    double center_y = 0;

    for (int s = 1; s <= N_step; ++s) {
      double angle, step_length, lateral_offset;

      if (!step_directions.empty() && s <= static_cast<int>(step_directions.size())) {
        // Use custom step directions
        std::tie(angle, step_length, lateral_offset) = step_directions[s - 1];
        current_angle = angle;

        // Determine lateral offset based on constraints
        lateral_offset = std::max(min_foot_distance_y / 2, std::min(lateral_offset, max_foot_distance_y / 2));
      }
      else {
        // Use default pattern with specified angle
        angle = step_angle;

        // Adjust step length to meet constraints
        step_length = std::max(min_foot_distance_x, std::min(sx, max_foot_distance_x));
        current_angle = angle;

        // Determine lateral offset based on constraints
        lateral_offset = std::max(min_foot_distance_y / 2, std::min(sy / 2, max_foot_distance_y / 2));
      }

      // Calculate forward movement components
      double dx = step_length * std::cos(angle);
      double dy = step_length * std::sin(angle);

      // Update the robot's center position
      center_x += dx;
      center_y += dy;

      double foot_x, foot_y;
      if (is_left) {
        // Left foot position calculation
        foot_x = center_x + lateral_offset * std::sin(-angle);
        foot_y = center_y + lateral_offset * std::cos(angle);
      }
      else {
        // Right foot position calculation
        foot_x = center_x - lateral_offset * std::sin(-angle);
        foot_y = center_y - lateral_offset * std::cos(angle);
      }

      // Add the new footstep
      Footstep new_step;
      new_step.position = Eigen::Vector2d(foot_x, foot_y);
      new_step.orientation = current_angle;
      new_step.foot = is_left ? "left" : "right";
      footsteps.push_back(new_step);

      is_left = !is_left;  // Alternate feet
    }

    return footsteps;
  }

  Eigen::MatrixXd WalkPattern3DGenerator::create_ZMP_pattern(double sx, double sy, int N_step, double T_step, double dT) {
    // Generate footsteps if not already generated
    if (footsteps.empty()) {
      footsteps = generate_footsteps(sx, sy, N_step);
    }

    // Number of samples per step
    int N_sample = static_cast<int>(T_step / dT);

    // Idle in the first and last step
    Eigen::MatrixXd patterns = Eigen::MatrixXd::Zero((N_step + 2) * N_sample, 2);

    // Idle in the first step - ZMP at the center between feet
    double initial_center_x = (footsteps[0].position(0) + footsteps[1].position(0)) / 2;
    double initial_center_y = (footsteps[0].position(1) + footsteps[1].position(1)) / 2;

    for (int i = 0; i < N_sample; ++i) {
      patterns(i, 0) = initial_center_x;
      patterns(i, 1) = initial_center_y;
    }

    // Generate ZMP trajectory from footsteps
    for (int s = 1; s <= N_step; ++s) {
      const Footstep& footstep = footsteps[s + 1];  // +1 because we have the initial stance

      for (int i = 0; i < N_sample; ++i) {
        patterns(s * N_sample + i, 0) = footstep.position(0);
        patterns(s * N_sample + i, 1) = footstep.position(1);
      }
    }

    // Idle in the last step - use the final footstep position
    const Footstep& final_footstep = footsteps.back();
    for (int i = 0; i < N_sample; ++i) {
      patterns((N_step + 1) * N_sample + i, 0) = final_footstep.position(0);
      patterns((N_step + 1) * N_sample + i, 1) = final_footstep.position(1);
    }

    return patterns;
  }

  std::vector<std::tuple<double, double, double>> WalkPattern3DGenerator::create_omnidirectional_pattern(
    const std::vector<std::tuple<double, double, double>>& step_sequence) {
    step_directions = step_sequence;
    N_step = step_directions.size();
    // Clear any previously generated footsteps
    footsteps.clear();
    return step_directions;
  }

  void WalkPattern3DGenerator::set_walking_direction(double angle) {
    step_angle = angle;
  }

  std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
    WalkPattern3DGenerator::create_system(double dT, double zc, double g) {
    Eigen::MatrixXd A(3, 3);
    A << 1, dT, dT* dT / 2,
      0, 1, dT,
      0, 0, 1;

    Eigen::MatrixXd B(3, 1);
    B << dT * dT * dT / 6, dT* dT / 2, dT;

    Eigen::MatrixXd C(1, 3);
    C << 1, 0, -zc / g;

    return std::make_tuple(A, B, C);
  }

  Eigen::MatrixXd WalkPattern3DGenerator::solve_discrete_are(
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {

    // Initialize P with Q
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_prev;

    // Iterative solution
    double tolerance = 1e-10;
    int max_iterations = 1000;

    for (int i = 0; i < max_iterations; ++i) {
      P_prev = P;

      Eigen::MatrixXd temp = B.transpose() * P * B + R;
      Eigen::MatrixXd K = temp.inverse() * B.transpose() * P * A;

      P = A.transpose() * P * A - A.transpose() * P * B * K + Q;

      if ((P - P_prev).norm() < tolerance) {
        break;
      }
    }

    return P;
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
    WalkPattern3DGenerator::create_controller(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& C, double r, double q, int N) {
    Eigen::MatrixXd R = r * Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd Q = q * C.transpose() * C;

    // Solve the discrete algebraic Riccati equation
    Eigen::MatrixXd P = solve_discrete_are(A, B, Q, R);

    // Calculate K
    Eigen::MatrixXd tmp = (R + B.transpose() * P * B).inverse() * B.transpose();
    Eigen::MatrixXd K = tmp * P * A;

    // Calculate Fs
    Eigen::VectorXd Fs(N);
    Eigen::MatrixXd pre = tmp;
    Eigen::MatrixXd AcT = (A - B * K).transpose();

    for (int n = 0; n < N; ++n) {
      Fs(n) = (pre * C.transpose() * q)(0, 0);
      pre = pre * AcT;
    }

    return std::make_tuple(K, Fs);
  }

  std::tuple<Eigen::VectorXd, double>
    WalkPattern3DGenerator::update_state(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& C, const Eigen::VectorXd& X,
      const Eigen::VectorXd& U) {
    Eigen::VectorXd X_next = A * X + B * U;
    double P_k = (C * X)(0);
    return std::make_tuple(X_next, P_k);
  }

  std::tuple<Eigen::VectorXd, Eigen::VectorXd>
    WalkPattern3DGenerator::solve_system(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& C, const Eigen::MatrixXd& K,
      const Eigen::VectorXd& Fs, const Eigen::VectorXd& pref,
      int N, double x0, double dx0, double d2x0) {
    // Initial state vector
    Eigen::VectorXd X(3);
    X << x0, dx0, d2x0;

    int n_zmps = pref.size();
    Eigen::VectorXd pref_tmp(n_zmps + N - 1);
    pref_tmp.head(n_zmps) = pref;
    for (int i = 0; i < N - 1; ++i) {
      pref_tmp(n_zmps + i) = pref(n_zmps - 1);
    }

    Eigen::VectorXd Xs(n_zmps);
    Eigen::VectorXd ZMPs(n_zmps);

    for (int i = 0; i < n_zmps; ++i) {
      Eigen::VectorXd pref_window = pref_tmp.segment(i, N);
      double U_val = -(K * X)(0, 0) + Fs.dot(pref_window);

      Eigen::VectorXd U(1);
      U(0) = U_val;

      Eigen::VectorXd X_next;
      double ZMP;
      std::tie(X_next, ZMP) = update_state(A, B, C, X, U);

      X = X_next;
      Xs(i) = X(0);
      ZMPs(i) = ZMP;
    }

    return std::make_tuple(Xs, ZMPs);
  }

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>
    WalkPattern3DGenerator::generate(double x0, double dx0, double d2x0, double y0, double dy0, double d2y0) {
    // Create the controller
    Eigen::MatrixXd A, B, C;
    std::tie(A, B, C) = create_system(dT, zc, g);

    Eigen::MatrixXd K;
    Eigen::VectorXd Fs;
    std::tie(K, Fs) = create_controller(A, B, C, r, q, N);

    // Generate the footsteps first
    footsteps = generate_footsteps(sx, sy, N_step);

    // Generate the ZMPs based on footsteps
    Eigen::MatrixXd pref = create_ZMP_pattern(sx, sy, N_step, T_step, dT);
    Eigen::VectorXd prefx = pref.col(0);
    Eigen::VectorXd prefy = pref.col(1);

    // With given ZMPs, compute the trajectory of CoM
    Eigen::VectorXd Xs, Pxs, Ys, Pys;
    std::tie(Xs, Pxs) = solve_system(A, B, C, K, Fs, prefx, N, x0, dx0, d2x0);
    std::tie(Ys, Pys) = solve_system(A, B, C, K, Fs, prefy, N, y0, dy0, d2y0);

    return std::make_tuple(Xs, Ys, Pxs, Pys, prefx, prefy);
  }

  // NEW: Method to read COM sensor data
  Eigen::Vector3d WalkPattern3DGenerator::read_com_sensor_data() {
    // In a real implementation, this would interface with hardware sensors
    std::cout << "Reading COM sensor data..." << std::endl;

    // Placeholder for sensor data acquisition
    // This could involve:
    // 1. Reading from ROS topics
    // 2. Accessing sensor APIs
    // 3. Processing IMU and force sensor data

    // For now, just return current stored position
    return com_position;
  }

  // NEW: Method to read joint encoder data
  JointEncoderData WalkPattern3DGenerator::read_joint_encoder_data() {
    std::cout << "Reading joint encoder data..." << std::endl;

    // In a real implementation, this would read from the robot's encoders
    JointEncoderData data;

    // Example with a humanoid robot (simplified)
    // Populate with real encoder data in an actual implementation
    data.joint_names = {
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
        "left_knee", "left_ankle_pitch", "left_ankle_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
        "right_knee", "right_ankle_pitch", "right_ankle_roll"
    };

    // Initialize with zeros - in a real implementation, these would be actual readings
    data.joint_angles.resize(data.joint_names.size(), 0.0);

    return data;
  }

  // NEW: Method to calculate COM from joint encoders
  Eigen::Vector3d WalkPattern3DGenerator::calculate_com_from_joint_encoders(const JointEncoderData& joint_data) {
    std::cout << "Calculating COM position from joint encoders..." << std::endl;

    // In a real implementation, this would use forward kinematics
    // to calculate the COM position based on joint angles

    // Simplified example - in reality, this would be a complex forward kinematics calculation
    // using the robot's URDF model and mass distribution

    // For demonstration purposes - create a basic COM estimate
    Eigen::Vector3d calculated_com = Eigen::Vector3d::Zero();

    // In a real implementation:
    // 1. Compute forward kinematics for each link
    // 2. Calculate center of mass of each link
    // 3. Compute weighted average based on mass to get overall COM

    // Placeholder for real implementation
    if (!joint_data.joint_angles.empty()) {
      // Example - in reality, this would be based on the robot's kinematic model
      // This is just a simplified placeholder

      // Use hip joints as a reference point (simplified)
      int left_hip_pitch_idx = 2;
      int right_hip_pitch_idx = 8;

      if (left_hip_pitch_idx < static_cast<int>(joint_data.joint_angles.size()) &&
        right_hip_pitch_idx < static_cast<int>(joint_data.joint_angles.size())) {

        double hip_pitch_avg = (joint_data.joint_angles[left_hip_pitch_idx] +
          joint_data.joint_angles[right_hip_pitch_idx]) / 2.0;

        // Simplified COM calculation (in real application, use proper kinematics)
        calculated_com(0) = 0.0;  // X position
        calculated_com(1) = 0.0;  // Y position
        calculated_com(2) = zc - 0.05 * std::cos(hip_pitch_avg);  // Z position with slight adjustment
      }
    }

    return calculated_com;
  }

  // NEW: Method to update the walking pattern using sensor data
  void WalkPattern3DGenerator::update_from_sensors() {
    // 1. Read COM sensor data
    Eigen::Vector3d sensor_com = read_com_sensor_data();

    // 2. Read joint encoder data
    JointEncoderData joint_data = read_joint_encoder_data();

    // 3. Calculate COM from joint encoders as validation/backup
    Eigen::Vector3d encoder_com = calculate_com_from_joint_encoders(joint_data);

    // 4. Decide which COM estimate to use (sensor or encoder-based)
    // In a real implementation, you might use a sensor fusion algorithm
    // For now, we'll use a simple heuristic - prefer sensor data if available
    Eigen::Vector3d final_com;

    // Simple check if sensor data seems valid
    if (sensor_com.norm() > 1e-6) {
      final_com = sensor_com;
      std::cout << "Using COM sensor data" << std::endl;
    }
    else {
      final_com = encoder_com;
      std::cout << "Using encoder-based COM calculation" << std::endl;
    }

    // 5. Update the internal COM position
    set_com_position(final_com);

    // 6. In a real implementation, we might regenerate the walking pattern here
    // based on the updated COM position
  }

  // NEW: Method to update the COM position
  void WalkPattern3DGenerator::set_com_position(const Eigen::Vector3d& new_com_position) {
    com_position = new_com_position;

    std::cout << "Updated COM position to: ["
      << com_position(0) << ", "
      << com_position(1) << ", "
      << com_position(2) << "]" << std::endl;
  }

} // namespace zmp_walk
