#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include "lbfgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ghost planner/mocha_types.hpp"

namespace ghost_planner {

class MochaOptimizer {
public:

  struct LbfgsData {
    const MochaParameters* params;
    rclcpp::Logger logger;
    Eigen::MatrixXd M_banded;
    Eigen::SparseMatrix<double> M_full;
  };

  static bool optimize(const MochaParameters& params, MochaTrajectory& trajectory, rclcpp::Logger logger);
  static double costFunctionVector(const Eigen::VectorXd& x_vec, Eigen::VectorXd* g_out, void* data);
  static double lbfgsEvaluate(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g);

  static void buildMochaMatrix(const std::vector<Eigen::Vector2d>& waypoints,
                             const Eigen::VectorXd& T,
                             const MochaParameters& params,
                             Eigen::MatrixXd& M_banded,
                             Eigen::MatrixXd& b,
                             Eigen::SparseMatrix<double>& M_full);

  static Eigen::MatrixXd solveBandedSystem(Eigen::MatrixXd& M_banded,
                                           const Eigen::MatrixXd& b,
                                           int p, int q);

  static Eigen::MatrixXd solveBandedSystemAdj(const Eigen::MatrixXd& M_banded,
                                              const Eigen::MatrixXd& b,
                                              int p, int q);

  static void calculateEnergyAndGradient(const Eigen::MatrixXd& coeffs,
                                         const Eigen::VectorXd& T,
                                         const MochaParameters& params,
                                         double& energy,
                                         Eigen::MatrixXd& grad_c,
                                         Eigen::VectorXd& grad_T);

  static void calculatePenaltyAndGradient(const Eigen::MatrixXd& coeffs,
                                          const Eigen::VectorXd& T,
                                          const MochaParameters& params,
                                          double& penalty,
                                          Eigen::MatrixXd& grad_c,
                                          Eigen::VectorXd& grad_T);

  static void backpropagateMochaGradient(const Eigen::VectorXd& T,
                                       const Eigen::MatrixXd& total_grad_c,
                                       const Eigen::VectorXd& total_grad_T_direct,
                                       const MochaParameters& params,
                                       const Eigen::MatrixXd& M_banded_LU,
                                       const Eigen::MatrixXd& coeffs,
                                       Eigen::MatrixXd& grad_q,
                                       Eigen::VectorXd& grad_T_total);

  static Eigen::VectorXd backpropagateGradQtoV(const Eigen::MatrixXd& grad_q,
                                               const MochaParameters& params,
                                               rclcpp::Logger logger);

  static Eigen::VectorXd forwardT(const Eigen::VectorXd& tau);
  static Eigen::VectorXd backwardT(const Eigen::VectorXd& T);
  static Eigen::VectorXd backwardGradT(const Eigen::VectorXd& tau, const Eigen::VectorXd& gradT);

  static double v_from_s(double s, double v_min = 0.5, double v_max = 1.5);
  static double dvds_from_s(double s, double v_min = 0.5, double v_max = 1.5);
  static double s_from_v(double v, double v_min = 0.5, double v_max = 1.5);

  static void getPolyBases(double t, int n_order,
                           Eigen::VectorXd& B0, Eigen::VectorXd& B1,
                           Eigen::VectorXd& B2, Eigen::VectorXd& B3);
  static Eigen::VectorXd getPolyBasis(double t, int n_order, int derivative_order);
};

} // namespace ghost_planner
