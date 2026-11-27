#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <tuple>
#include <cmath>

namespace zmp_walk {

/**
 * Implementation of ZMP-based walking pattern generation.
 * This is a C++ port of the Python ZMP walk pattern generator.
 * 
 * Background:
 * ZMP (Zero Moment Point) is a method for making a walking robot keep balance.
 * The goal of this implementation is to deduce the trajectory of CoM with given ZMPs.
 * 
 * For more details, refer to "Introduction to Humanoid Robotics" by Shuuji Kajita.
 */

// Structure to represent a footstep
struct Footstep {
    Eigen::Vector2d position;
    double orientation;
    std::string foot; // "left" or "right"
};

// Joint encoder data structure
struct JointEncoderData {
    std::vector<double> joint_angles;
    std::vector<std::string> joint_names;
};

class WalkPattern3DGenerator {
public:
    // Constructor with default parameters
    WalkPattern3DGenerator(
        double sx = 0.3, double sy = 0.1, int N_step = 5, 
        double T_step = 1.0, double dT = 5e-3, double zc = 0.8, 
        double q = 1.0, double r = 1e-6, double g = 9.81, 
        int N = 320, double step_angle = 0.0, 
        const std::vector<std::tuple<double, double, double>>& step_directions = {});

    // Footstep generation methods
    std::vector<Footstep> generate_footsteps(double sx, double sy, int N_step);
    Eigen::MatrixXd create_ZMP_pattern(double sx, double sy, int N_step, double T_step, double dT);
    std::vector<std::tuple<double, double, double>> create_omnidirectional_pattern(
        const std::vector<std::tuple<double, double, double>>& step_sequence);
    void set_walking_direction(double angle);

    // System dynamics and control methods
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> create_system(double dT, double zc, double g);
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd> create_controller(
        const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, 
        double r, double q, int N);
    std::tuple<Eigen::VectorXd, double> update_state(
        const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, 
        const Eigen::VectorXd& X, const Eigen::VectorXd& U);
    std::tuple<Eigen::VectorXd, Eigen::VectorXd> solve_system(
        const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& K, const Eigen::VectorXd& Fs,
        const Eigen::VectorXd& pref, int N, double x0, double dx0, double d2x0);

    // Main generation method
    std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> 
    generate(double x0 = 0.0, double dx0 = 0.0, double d2x0 = 0.0, 
             double y0 = 0.0, double dy0 = 0.0, double d2y0 = 0.0);

    // NEW: Sensor data reading and COM calculation methods
    Eigen::Vector3d read_com_sensor_data();
    JointEncoderData read_joint_encoder_data();
    Eigen::Vector3d calculate_com_from_joint_encoders(const JointEncoderData& joint_data);
    
    // NEW: Update the walking pattern using sensor data
    void update_from_sensors();
    void set_com_position(const Eigen::Vector3d& com_position);

private:
    // Walking pattern parameters
    double sx, sy;
    int N_step;
    double T_step, dT;
    double step_angle;
    std::vector<std::tuple<double, double, double>> step_directions;
    
    // Walking phase parameters
    double double_support_ratio;
    double swing_ratio;
    
    // Foot placement constraints
    double min_foot_distance_x;
    double max_foot_distance_x;
    double min_foot_distance_y;
    double max_foot_distance_y;
    
    // Preview control parameters
    double zc;
    double q;
    double r;
    double g;
    int N;
    
    // Foot dimensions
    double foot_length;
    double foot_width;
    
    // Storage for generated footsteps
    std::vector<Footstep> footsteps;
    
    // NEW: COM state tracking variables
    Eigen::Vector3d com_position;
    Eigen::Vector3d com_velocity;
    Eigen::Vector3d com_acceleration;

    // Helper function for discrete algebraic Riccati equation
    Eigen::MatrixXd solve_discrete_are(
        const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
        const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
};

} // namespace zmp_walk
