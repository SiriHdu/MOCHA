#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <rclcpp/logger.hpp>

namespace ghost_planner
{

struct MochaParameters
{
    // Dynamics constraints
    double v_max = 2.5;
    double a_max = 5.0;
    double drone_radius = 1.0;
    std::vector<Eigen::Vector3d> obstacles;  // [x, y, radius]

    // Trajectory parameters
    const int dims = 2;
    const int s = 3;
    const int n_order = 2 * s - 1;  // 5
    const int n_coeffs = n_order + 1;  // 6

    // Optimization weights
    double w_energy = 1.0;
    double w_time = 1.0;
    double w_feasibility = 10.0;
    double w_obstacle = 100.0;

    // Sampling
    int kappa = 10;

    // Boundary conditions
    Eigen::Vector2d start_waypoint;
    Eigen::Vector2d end_waypoint;
    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_acc = Eigen::Vector2d::Zero();

    // Segment allocation
    int n_segments;
    std::vector<double> initial_segment_times;

    // Motion camouflage parameters
    Eigen::Vector2d ref_point = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> prey_points;
    std::vector<double> initial_v;

    // Multi-agent collision avoidance
    struct PeerSampledTraj {
        double radius{0.2};
        double plan_start_sec{0.0};
        int dims{2};
        int n_coeffs{6};
        int n_segments{0};
        Eigen::MatrixXd coeffs;
        Eigen::VectorXd segment_T;
        bool has_poly{false};
    };
    std::vector<PeerSampledTraj> peer_trajs;
    double w_peer{200.0};
    double peer_safety_margin{0.10};
    double self_plan_start_sec{0.0};
};

struct NloptData
{
    const MochaParameters* params;
    rclcpp::Logger logger;
    Eigen::MatrixXd M_banded;
    Eigen::SparseMatrix<double> M_full;
};

struct MochaTrajectory
{
    Eigen::MatrixXd coeffs;
    Eigen::VectorXd T;
    double total_duration;
    double final_cost{0.0};

    bool isValid() const {
        return T.size() > 0 && coeffs.size() > 0;
    }
};

} // namespace ghost_planner
