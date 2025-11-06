#include "ghost planner/mocha_optimizer.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace ghost_planner {


double MochaOptimizer::v_from_s(double s, double v_min, double v_max) {
  const double y = 1.0 / (1.0 + std::exp(-s));
  return v_min + (v_max - v_min) * y;
}
double MochaOptimizer::dvds_from_s(double s, double v_min, double v_max) {
  const double y = 1.0 / (1.0 + std::exp(-s));
  return (v_max - v_min) * y * (1.0 - y);
}
double MochaOptimizer::s_from_v(double v, double v_min, double v_max) {
  double y = (v - v_min) / (v_max - v_min);
  const double eps = 1e-12;
  if (y <= eps) y = eps;
  if (y >= 1.0 - eps) y = 1.0 - eps;
  return std::log(y / (1.0 - y));
}

static inline double pointToSegmentDistanceHelper(const Eigen::RowVector2d& p,
                                                  const Eigen::RowVector2d& a,
                                                  const Eigen::RowVector2d& b) {
  Eigen::RowVector2d ab = b - a;
  double denom = ab.squaredNorm();
  if (denom < 1e-12) {
    return (p - a).norm();
  }
  double t = (p - a).dot(ab) / denom;
  t = std::max(0.0, std::min(1.0, t));
  Eigen::RowVector2d proj = a + t * ab;
  return (p - proj).norm();
}


Eigen::VectorXd MochaOptimizer::backpropagateGradQtoV(const Eigen::MatrixXd& grad_q,
                                                    const MochaParameters& params,
                                                    rclcpp::Logger logger) {
  const int n_intermediate = params.n_segments > 1 ? params.n_segments - 1 : 0;
  Eigen::VectorXd grad_v = Eigen::VectorXd::Zero(n_intermediate);

  for (int i = 0; i < n_intermediate; ++i) {
    if ((int)params.prey_points.size() <= i + 1) {
      RCLCPP_ERROR(logger, "prey_points insufficient in backpropagateGradQtoV. Need %d, got %zu", n_intermediate + 1, params.prey_points.size());
      return grad_v; // zeros
    }
    Eigen::Vector2d prey_i = params.prey_points[i + 1];
    Eigen::Vector2d dq_dv = prey_i - params.ref_point;
    double gv = 0.0;
    if (grad_q.cols() >= 2) {
      gv = grad_q(i, 0) * dq_dv.x() + grad_q(i, 1) * dq_dv.y();
    }
    grad_v(i) = gv;
  }
  return grad_v;
}


bool MochaOptimizer::optimize(const MochaParameters& params, MochaTrajectory& trajectory, rclcpp::Logger logger)
{
  const int n_segments = params.n_segments;
  const int s_len = std::max(0, n_segments - 1);

  if ((int)params.prey_points.size() < n_segments + 1) {
    RCLCPP_ERROR(logger, "prey_points size insufficient. Expected %d, got %zu", n_segments + 1, params.prey_points.size());
    return false;
  }

  Eigen::VectorXd T_initial(n_segments);
  if ((int)params.initial_segment_times.size() == n_segments) {
    for (int i = 0; i < n_segments; ++i) {
      T_initial(i) = std::max(1e-6, params.initial_segment_times[i]);
    }
  } else {
    const double path_length = (params.start_waypoint - params.end_waypoint).norm();
    const double avg_segment_length = (n_segments > 0) ? (path_length / n_segments) : 0.0;
    const double reasonable_speed = std::max(1e-6, std::min(params.v_max * 0.5, 2.0));
    double time_per_segment = std::max(0.5, std::min(3.0, avg_segment_length / reasonable_speed));
    double initial_total_time = std::max(n_segments * 0.5, std::min(n_segments * 3.0, time_per_segment * n_segments));
    time_per_segment = (n_segments > 0) ? (initial_total_time / n_segments) : 0.0;
    T_initial = Eigen::VectorXd::Ones(n_segments) * time_per_segment;
  }
  Eigen::VectorXd tau_initial = backwardT(T_initial);

  Eigen::VectorXd v_initial(s_len);
  if ((int)params.initial_v.size() == s_len) {
    for (int i = 0; i < s_len; ++i) v_initial(i) = params.initial_v[i];
  } else {
    v_initial.setOnes();
    RCLCPP_WARN(logger, "initial_v size mismatch! Expect %d, got %zu. Using all 1.0", s_len, params.initial_v.size());
  }
  Eigen::VectorXd s_initial(s_len);
  for (int i = 0; i < s_len; ++i) s_initial(i) = MochaOptimizer::s_from_v(v_initial(i));

  RCLCPP_DEBUG(logger, "\n=== MOCHA INIT ===");
  {
    std::stringstream ss_T, ss_vmapped;
    for (int i = 0; i < T_initial.size(); ++i) {
      ss_T << std::fixed << std::setprecision(6) << T_initial(i) << (i+1<T_initial.size()? ", ":"");
    }
    for (int i = 0; i < s_initial.size(); ++i) {
      ss_vmapped << std::fixed << std::setprecision(6) << MochaOptimizer::v_from_s(s_initial(i))
                 << (i+1<s_initial.size()? ", ":"");
    }
    RCLCPP_DEBUG(logger, "Initial T: [%s]", ss_T.str().c_str());
    RCLCPP_DEBUG(logger, "Initial v: [%s]", ss_vmapped.str().c_str());
  }

  Eigen::VectorXd x(n_segments + s_len);
  x.head(n_segments) = tau_initial;
  if (s_len > 0) x.tail(s_len) = s_initial;

  LbfgsData data_wrapper{&params, logger, Eigen::MatrixXd(), Eigen::SparseMatrix<double>()};
  lbfgs::lbfgs_parameter_t lbfgs_param;
  lbfgs_param.max_iterations = 80;
  lbfgs_param.g_epsilon = 1e-6;
  lbfgs_param.mem_size=10;
  lbfgs_param.max_linesearch=64;
  lbfgs_param.f_dec_coeff=1e-4;
  lbfgs_param.s_curv_coeff=0.9;

  double final_cost = 0.0;
  auto fg = [&](const Eigen::VectorXd& xvec, Eigen::VectorXd& gvec) -> double {
    return MochaOptimizer::costFunctionVector(xvec, &gvec, &data_wrapper);
  };
  struct CallbackData { decltype(fg)* fn; } cbd{ &fg };
  auto eval = [](void* instance, const Eigen::VectorXd& x_in, Eigen::VectorXd& g_out) -> double {
    auto* data = reinterpret_cast<CallbackData*>(instance);
    return (*(data->fn))(x_in, g_out);
  };

  Eigen::VectorXd xvec = x;
  auto t0 = std::chrono::high_resolution_clock::now();
  int ret = lbfgs::lbfgs_optimize(xvec, final_cost, eval, nullptr, nullptr, &cbd, lbfgs_param);
  RCLCPP_DEBUG(logger, "L-BFGS finished with code %d", ret);
  auto t1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt = t1 - t0;
  RCLCPP_DEBUG(logger, "MOCHA optimization time: %.4fs", dt.count());
  x = xvec;

  Eigen::VectorXd tau_opt = x.head(n_segments);
  Eigen::VectorXd s_opt  = s_len > 0 ? x.tail(s_len) : Eigen::VectorXd();
  trajectory.T = forwardT(tau_opt);
  trajectory.final_cost = final_cost;

  {
    std::stringstream ss_T_opt, ss_v_opt;
    for (int i = 0; i < trajectory.T.size(); ++i) {
      ss_T_opt << std::fixed << std::setprecision(6) << trajectory.T(i) << (i+1<trajectory.T.size()? ", ":"");
    }
    for (int i = 0; i < s_opt.size(); ++i) {
      ss_v_opt << std::fixed << std::setprecision(6) << MochaOptimizer::v_from_s(s_opt(i))
               << (i+1<s_opt.size()? ", ":"");
    }
    RCLCPP_DEBUG(logger, "Optimized T: [%s]", ss_T_opt.str().c_str());
    if (s_opt.size() > 0) RCLCPP_DEBUG(logger, "Optimized v: [%s]", ss_v_opt.str().c_str());
  }

  std::vector<Eigen::Vector2d> final_wps;
  final_wps.reserve(n_segments + 1);
  final_wps.push_back(params.start_waypoint);
  for (int i = 0; i < s_len; ++i) {
    Eigen::Vector2d prey_i = params.prey_points[i + 1];
    const double v_i = MochaOptimizer::v_from_s(s_opt(i));
    Eigen::Vector2d q_i = params.ref_point + v_i * (prey_i - params.ref_point);
    final_wps.push_back(q_i);
  }
  final_wps.push_back(params.end_waypoint);

  Eigen::MatrixXd M_banded_final, b_final;
  Eigen::SparseMatrix<double> M_full_final;
  buildMochaMatrix(final_wps, trajectory.T, params, M_banded_final, b_final, M_full_final);
  trajectory.coeffs = solveBandedSystem(M_banded_final, b_final, params.n_coeffs, params.n_coeffs);
  trajectory.total_duration = trajectory.T.sum();
  return true;
}


double MochaOptimizer::costFunctionVector(const Eigen::VectorXd& x_vec, Eigen::VectorXd* g_out, void* data)
{
  LbfgsData* d = reinterpret_cast<LbfgsData*>(data);
  const MochaParameters& params = *d->params;
  const int n_segments = params.n_segments;
  const int s_len = std::max(0, n_segments - 1);

  const int tau_len = n_segments;

  Eigen::VectorXd tau = x_vec.head(tau_len);
  Eigen::VectorXd s_vec = s_len > 0 ? x_vec.tail(s_len) : Eigen::VectorXd();

  Eigen::VectorXd v_vec(s_len), dvds(s_len);
  for (int i = 0; i < s_len; ++i) {
    v_vec(i) = MochaOptimizer::v_from_s(s_vec(i));
    dvds(i)  = MochaOptimizer::dvds_from_s(s_vec(i));
  }
  Eigen::VectorXd T = forwardT(tau);

  if (!tau.allFinite() || !T.allFinite() || (s_len>0 && !s_vec.allFinite())) {
    if (g_out) g_out->setZero(x_vec.size());
    RCLCPP_ERROR(d->logger, "Non-finite variables in costFunction.");
    return 1e10;
  }

  std::vector<Eigen::Vector2d> waypoints;
  waypoints.reserve(n_segments + 1);
  waypoints.push_back(params.start_waypoint);
  for (int i = 0; i < s_len; ++i) {
    if ((int)params.prey_points.size() <= i + 1) {
      if (g_out) g_out->setZero(x_vec.size());
      RCLCPP_ERROR(d->logger, "prey_points size insufficient (costFunctionVector).");
      return 1e10;
    }
    Eigen::Vector2d prey_i = params.prey_points[i + 1];
    const double v_i = v_vec(i);
    Eigen::Vector2d q_i = params.ref_point + v_i * (prey_i - params.ref_point);
    waypoints.push_back(q_i);
    if (!q_i.allFinite()) {
      if (g_out) g_out->setZero(x_vec.size());
      RCLCPP_ERROR(d->logger, "Non-finite waypoint q_%d.", i);
      return 1e10;
    }
  }
  waypoints.push_back(params.end_waypoint);

  Eigen::MatrixXd b;
  buildMochaMatrix(waypoints, T, params, d->M_banded, b, d->M_full);
  Eigen::MatrixXd M_banded_factor = d->M_banded;
  Eigen::MatrixXd coeffs = solveBandedSystem(M_banded_factor, b, params.n_coeffs, params.n_coeffs);
  if (coeffs.hasNaN()) {
    if (g_out) g_out->setZero(x_vec.size());
    RCLCPP_ERROR(d->logger, "Solve produced NaN (likely bad T).");
    return 1e9;
  }

  double energy_cost = 0.0, penalty_cost = 0.0;
  Eigen::MatrixXd grad_E_c, grad_P_c;
  Eigen::VectorXd grad_E_T, grad_P_T;
  calculateEnergyAndGradient(coeffs, T, params, energy_cost, grad_E_c, grad_E_T);
  calculatePenaltyAndGradient(coeffs, T, params, penalty_cost, grad_P_c, grad_P_T);
  const double time_cost = T.sum();
  const double total_cost = params.w_energy * energy_cost + penalty_cost + params.w_time * time_cost;

  if (g_out) {
    g_out->resize(tau_len + s_len);

    Eigen::MatrixXd total_grad_c = grad_E_c + grad_P_c;
    Eigen::VectorXd total_grad_T_direct = grad_E_T + grad_P_T + Eigen::VectorXd::Ones(n_segments);

    Eigen::MatrixXd grad_q;
    Eigen::VectorXd grad_T_total;
    backpropagateMochaGradient(T, total_grad_c, total_grad_T_direct, params, M_banded_factor, coeffs,
                             grad_q, grad_T_total);

    Eigen::VectorXd grad_tau = backwardGradT(tau, grad_T_total);
    Eigen::VectorXd grad_v = backpropagateGradQtoV(grad_q, params, d->logger);
    Eigen::VectorXd grad_s = grad_v.array() * dvds.array();

    g_out->head(tau_len) = grad_tau;
    if (s_len > 0) g_out->tail(s_len) = grad_s;
  }
  return total_cost;
}

double MochaOptimizer::lbfgsEvaluate(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g)
{
  return MochaOptimizer::costFunctionVector(x, &g, instance);
}


Eigen::VectorXd MochaOptimizer::forwardT(const Eigen::VectorXd& tau) {
  Eigen::VectorXd T(tau.size());
  for (int i = 0; i < tau.size(); ++i) {
    if (tau(i) > 0) {
      T(i) = (0.5 * tau(i) + 1.0) * tau(i) + 1.0;
    } else {
      T(i) = 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
  }
  return T;
}
Eigen::VectorXd MochaOptimizer::backwardT(const Eigen::VectorXd& T) {
  Eigen::VectorXd tau(T.size());
  for (int i = 0; i < T.size(); ++i) {
    double t_val = T(i);
    if (t_val > 1.0) {
      tau(i) = std::sqrt(2.0 * t_val - 1.0) - 1.0;
    } else {
      if (t_val < 1e-12) t_val = 1e-12;
      tau(i) = 1.0 - std::sqrt(2.0 / t_val - 1.0);
    }
  }
  return tau;
}
Eigen::VectorXd MochaOptimizer::backwardGradT(const Eigen::VectorXd& tau, const Eigen::VectorXd& gradT) {
  Eigen::VectorXd gradTau(tau.size());
  for (int i = 0; i < tau.size(); ++i) {
    double dT_dtau = 0.0;
    if (tau(i) > 0) dT_dtau = tau(i) + 1.0;
    else {
      const double den = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
      dT_dtau = -(tau(i) - 1.0) / (den * den);
    }
    gradTau(i) = gradT(i) * dT_dtau;
  }
  return gradTau;
}


void MochaOptimizer::getPolyBases(double t, int n_order,
                                Eigen::VectorXd& B0, Eigen::VectorXd& B1,
                                Eigen::VectorXd& B2, Eigen::VectorXd& B3) {
  int n_coeffs = n_order + 1;
  B0.setZero(n_coeffs);
  B1.setZero(n_coeffs);
  B2.setZero(n_coeffs);
  B3.setZero(n_coeffs);
  for (int i = 0; i <= n_order; ++i) B0(i) = std::pow(t, i);
  for (int i = 1; i <= n_order; ++i) B1(i) = i * std::pow(t, i - 1);
  for (int i = 2; i <= n_order; ++i) B2(i) = i * (i - 1) * std::pow(t, i - 2);
  for (int i = 3; i <= n_order; ++i) B3(i) = i * (i - 1) * (i - 2) * std::pow(t, i - 3);
}
Eigen::VectorXd MochaOptimizer::getPolyBasis(double t, int n_order, int derivative_order) {
  int n_coeffs = n_order + 1;
  Eigen::VectorXd basis = Eigen::VectorXd::Zero(n_coeffs);
  if (derivative_order > n_order) return basis;
  for (int i = derivative_order; i <= n_order; ++i) {
    double coeff = 1.0;
    for (int j = 0; j < derivative_order; ++j) coeff *= (i - j);
    basis(i) = coeff * std::pow(t, i - derivative_order);
  }
  return basis;
}


void MochaOptimizer::buildMochaMatrix(const std::vector<Eigen::Vector2d>& waypoints,
                                  const Eigen::VectorXd& T,
                                  const MochaParameters& params,
                                  Eigen::MatrixXd& M_banded,
                                  Eigen::MatrixXd& b,
                                  Eigen::SparseMatrix<double>& M_full) {
  const int n_segments = params.n_segments;
  const int n_coeffs = params.n_coeffs;
  const int total_vars = n_segments * n_coeffs;
  const int s = params.s;
  const int dims = params.dims;

  const int p = n_coeffs, q = n_coeffs;
  M_banded.setZero(p + q + 1, total_vars);
  b.setZero(total_vars, dims);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(10 * total_vars);

  auto set_val = [&](int r, int c, double val) {
    if (val != 0.0) {
      M_banded(r - c + q, c) = val;
      triplets.emplace_back(r, c, val);
    }
  };

  int r_base = 0;
  for (int k = 0; k < s; ++k) {
    Eigen::VectorXd basis = getPolyBasis(0.0, params.n_order, k);
    for (int j = 0; j < n_coeffs; ++j) set_val(r_base + k, j, basis(j));
  }
  b.row(0) = params.start_waypoint;
  b.row(1) = params.start_vel;
  b.row(2) = params.start_acc;
  r_base += s;

  for (int i = 0; i < n_segments - 1; ++i) {
    int c_base = i * n_coeffs;
    double t_i = T(i);

    for (int k = s; k < 2 * s - 1; ++k) {
      Eigen::VectorXd basis1 = getPolyBasis(t_i, params.n_order, k);
      Eigen::VectorXd basis2 = getPolyBasis(0.0, params.n_order, k);
      for (int j = 0; j < n_coeffs; ++j) {
        set_val(r_base, c_base + j, basis1(j));
        set_val(r_base, c_base + n_coeffs + j, -basis2(j));
      }
      r_base++;
    }
    Eigen::VectorXd basis_p = getPolyBasis(t_i, params.n_order, 0);
    for (int j = 0; j < n_coeffs; ++j) set_val(r_base, c_base + j, basis_p(j));
    b.row(r_base) = waypoints[i + 1];
    r_base++;

    for (int k = 0; k < s; ++k) {
      Eigen::VectorXd basis1 = getPolyBasis(t_i, params.n_order, k);
      Eigen::VectorXd basis2 = getPolyBasis(0.0, params.n_order, k);
      for (int j = 0; j < n_coeffs; ++j) {
        set_val(r_base, c_base + j, basis1(j));
        set_val(r_base, c_base + n_coeffs + j, -basis2(j));
      }
      r_base++;
    }
  }

  int c_base = (n_segments - 1) * n_coeffs;
  double t_M = T.tail(1)(0);
  for (int k = 0; k < s; ++k) {
    Eigen::VectorXd basis = getPolyBasis(t_M, params.n_order, k);
    for (int j = 0; j < n_coeffs; ++j) set_val(r_base + k, c_base + j, basis(j));
  }
  b.row(r_base + 0) = params.end_waypoint;
  b.row(r_base + 1) = params.end_vel;
  b.row(r_base + 2) = params.end_acc;

  M_full.resize(total_vars, total_vars);
  M_full.setFromTriplets(triplets.begin(), triplets.end());
}

Eigen::MatrixXd MochaOptimizer::solveBandedSystem(Eigen::MatrixXd& M_banded,
                                                const Eigen::MatrixXd& b,
                                                int p, int q) {
  int N = b.rows();
  if (N == 0) return Eigen::MatrixXd(0, b.cols());

  for (int k = 0; k < N - 1; ++k) {
    double pivot = M_banded(q, k);
    if (std::abs(pivot) < 1e-14) {
      return Eigen::MatrixXd::Constant(b.rows(), b.cols(), std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = k + 1; i < std::min(k + p + 1, N); ++i) {
      M_banded(i - k + q, k) /= pivot;
    }
    for (int j = k + 1; j < std::min(k + q + 1, N); ++j) {
      double val = M_banded(k - j + q, j);
      if (val != 0.0) {
        for (int i = k + 1; i < std::min(k + p + 1, N); ++i) {
          M_banded(i - j + q, j) -= M_banded(i - k + q, k) * val;
        }
      }
    }
  }

  Eigen::MatrixXd y = b;
  for (int k = 0; k < N; ++k) {
    for (int i = k + 1; i < std::min(k + p + 1, N); ++i) {
      y.row(i) -= M_banded(i - k + q, k) * y.row(k);
    }
  }
  Eigen::MatrixXd x = y;
  for (int k = N - 1; k >= 0; --k) {
    x.row(k) /= M_banded(q, k);
    for (int i = std::max(0, k - q); i < k; ++i) {
      x.row(i) -= M_banded(i - k + q, k) * x.row(k);
    }
  }
  return x;
}

static Eigen::MatrixXd solveBandedAdjInternal(const Eigen::MatrixXd& M_banded,
                                              const Eigen::MatrixXd& b,
                                              int p, int q) {
  const int N = b.rows();
  if (N == 0) return Eigen::MatrixXd(0, b.cols());

  Eigen::MatrixXd x = b;

  int iM;
  for (int j = 0; j <= N - 1; ++j) {
    x.row(j) /= M_banded(q, j);
    iM = std::min(j + q, N - 1);
    for (int i = j + 1; i <= iM; ++i) {
      double aji = M_banded(j - i + q, i);
      if (aji != 0.0) {
        x.row(i) -= aji * x.row(j);
      }
    }
  }
  for (int j = N - 1; j >= 0; --j) {
    iM = std::max(0, j - p);
    for (int i = iM; i <= j - 1; ++i) {
      double aji = M_banded(j - i + q, i);
      if (aji != 0.0) {
        x.row(i) -= aji * x.row(j);
      }
    }
  }
  return x;
}

Eigen::MatrixXd MochaOptimizer::solveBandedSystemAdj(const Eigen::MatrixXd& M_banded,
                                                   const Eigen::MatrixXd& b,
                                                   int p, int q) {
  return solveBandedAdjInternal(M_banded, b, p, q);
}


void MochaOptimizer::calculateEnergyAndGradient(const Eigen::MatrixXd& coeffs,
                                              const Eigen::VectorXd& T,
                                              const MochaParameters& params,
                                              double& energy,
                                              Eigen::MatrixXd& grad_c,
                                              Eigen::VectorXd& grad_T) {
  energy = 0.0;
  grad_c.setZero(coeffs.rows(), coeffs.cols());
  grad_T.setZero(T.size());

  const int n_coeffs = params.n_coeffs;
  const int s = params.s;

  for (int i = 0; i < params.n_segments; ++i) {
    const auto& c3 = coeffs.row(i * n_coeffs + s);
    const auto& c4 = coeffs.row(i * n_coeffs + s + 1);
    const auto& c5 = coeffs.row(i * n_coeffs + s + 2);

    const double t1 = T(i);
    const double t2 = t1 * t1;
    const double t3 = t2 * t1;
    const double t4 = t3 * t1;
    const double t5 = t4 * t1;

    energy += 36.0 * c3.squaredNorm() * t1 +
              144.0 * c3.dot(c4) * t2 +
              192.0 * c4.squaredNorm() * t3 +
              240.0 * c3.dot(c5) * t3 +
              720.0 * c4.dot(c5) * t4 +
              720.0 * c5.squaredNorm() * t5;

    grad_c.row(i * n_coeffs + s)     = 72.0 * c3 * t1 + 144.0 * c4 * t2 + 240.0 * c5 * t3;
    grad_c.row(i * n_coeffs + s + 1) = 144.0 * c3 * t2 + 384.0 * c4 * t3 + 720.0 * c5 * t4;
    grad_c.row(i * n_coeffs + s + 2) = 240.0 * c3 * t3 + 720.0 * c4 * t4 + 1440.0 * c5 * t5;

    grad_T(i) = 36.0 * c3.squaredNorm() +
                288.0 * c3.dot(c4) * t1 +
                576.0 * c4.squaredNorm() * t2 +
                720.0 * c3.dot(c5) * t2 +
                2880.0 * c4.dot(c5) * t3 +
                3600.0 * c5.squaredNorm() * t4;
  }
}

void MochaOptimizer::calculatePenaltyAndGradient(const Eigen::MatrixXd& coeffs,
                                               const Eigen::VectorXd& T,
                                               const MochaParameters& params,
                                               double& penalty,
                                               Eigen::MatrixXd& grad_c,
                                               Eigen::VectorXd& grad_T) {
  penalty = 0.0;
  grad_c.setZero(coeffs.rows(), coeffs.cols());
  grad_T.setZero(T.size());

  for (int i = 0; i < params.n_segments; ++i) {
    Eigen::MatrixXd c_i = coeffs.block(i * params.n_coeffs, 0, params.n_coeffs, params.dims);
    int kappa_i = params.kappa;
    double h = T(i) / (double)kappa_i;

    const double influence_margin = 0.5;
    Eigen::VectorXd B0_start = getPolyBasis(0.0, params.n_order, 0);
    Eigen::VectorXd B0_end   = getPolyBasis(T(i), params.n_order, 0);
    Eigen::RowVector2d p0 = (B0_start.transpose() * c_i);
    Eigen::RowVector2d p1 = (B0_end.transpose() * c_i);
    std::vector<int> active_obs_indices;
    active_obs_indices.reserve(16);
    for (int oi = 0; oi < static_cast<int>(params.obstacles.size()); ++oi) {
      const auto& obs = params.obstacles[oi];
      Eigen::RowVector2d c = obs.head<2>().transpose();
      double r_safe = obs.z() + params.drone_radius + influence_margin;
      double d_line = pointToSegmentDistanceHelper(c, p0, p1);
      if (d_line <= r_safe) {
        active_obs_indices.push_back(oi);
      }
    }

    // absolute time prefix up to the start of segment i
    double t_prefix_seg = 0.0;
    for (int kk = 0; kk < i; ++kk) t_prefix_seg += T(kk);

    for (int j = 0; j <= kappa_i; ++j) {
      double t = (double)j / (double)kappa_i * T(i);
      Eigen::VectorXd B0, B1, B2, B3;
      getPolyBases(t, params.n_order, B0, B1, B2, B3);

      Eigen::RowVector2d pos = (B0.transpose() * c_i);
      Eigen::RowVector2d vel = (B1.transpose() * c_i);
      Eigen::RowVector2d acc = (B2.transpose() * c_i);
      Eigen::RowVector2d jer = (B3.transpose() * c_i);

      double omg = (j == 0 || j == kappa_i) ? 0.5 : 1.0;

      // ---- peer collision avoidance (soft constraint) ----
      if (!params.peer_trajs.empty()) {
        // absolute time on self trajectory
        double t_abs = params.self_plan_start_sec + t_prefix_seg + t;
        for (const auto &peer : params.peer_trajs) {
          double t_peer = t_abs - peer.plan_start_sec;
          if (t_peer < 0.0) continue;

          // validate peer polynomial shape
          if (!(peer.has_poly && peer.n_segments > 0 &&
                peer.segment_T.size() == peer.n_segments &&
                peer.coeffs.rows() == peer.n_coeffs * peer.n_segments &&
                peer.coeffs.cols() == peer.dims)) {
            continue;
          }

          // locate peer segment
          double accum = 0.0; int seg_idx = -1;
          for (int si = 0; si < peer.n_segments; ++si) {
            double next_accum = accum + peer.segment_T(si);
            if (t_peer <= next_accum || si == peer.n_segments - 1) { seg_idx = si; t_peer -= accum; break; }
            accum = next_accum;
          }
          if (seg_idx < 0) continue;

          Eigen::MatrixXd c_peer_i = peer.coeffs.block(seg_idx * peer.n_coeffs, 0, peer.n_coeffs, peer.dims);
          double clamped_t_peer = std::max(0.0, std::min(t_peer, peer.segment_T(seg_idx)));
          Eigen::VectorXd B0p = getPolyBasis(clamped_t_peer, params.n_order, 0);
          Eigen::VectorXd B1p = getPolyBasis(clamped_t_peer, params.n_order, 1);
          Eigen::RowVector2d p_peer = (B0p.transpose() * c_peer_i);
          Eigen::RowVector2d v_peer = (B1p.transpose() * c_peer_i);

          // distance-based penalty
          double r_safe = params.drone_radius + peer.radius + params.peer_safety_margin;
          double safe_dist_sq = r_safe * r_safe;
          Eigen::RowVector2d dvec = pos - p_peer;
          double dist_sq = dvec.squaredNorm();
          double coll_vio = safe_dist_sq - dist_sq;
          if (coll_vio > 0.0) {
            double cost_peer = params.w_peer * std::pow(coll_vio, 3);
            Eigen::RowVector2d grad_p_peer = params.w_peer * (-6.0) * std::pow(coll_vio, 2) * dvec;

            penalty += omg * h * cost_peer;
            grad_c.block(i * params.n_coeffs, 0, params.n_coeffs, params.dims) += omg * h * (B0 * grad_p_peer);

            // time gradients: weight term + absolute-time chain (prefix + current)
            double alpha = (double)j / (double)kappa_i;
            double dJ_dtabs_peer = grad_p_peer.dot(vel - v_peer);
            grad_T(i) += omg * (cost_peer / (double)kappa_i); // weight term
            for (int l = 0; l < i; ++l) grad_T(l) += omg * h * dJ_dtabs_peer; // all prefixes
            grad_T(i) += omg * h * (alpha * dJ_dtabs_peer); // current partial
          }
        }
      }

      for (const auto& obs_idx : active_obs_indices) {
        const auto& obs = params.obstacles[obs_idx];
        Eigen::RowVector2d obs_center = obs.head<2>().transpose();
        double safe_dist_sq = std::pow(obs.z() + params.drone_radius, 2);
        Eigen::RowVector2d dist_vec = pos - obs_center;
        double dist_sq = dist_vec.squaredNorm();
        double coll_vio = safe_dist_sq - dist_sq;
        if (coll_vio > 0) {
          double cost_obs = params.w_obstacle * std::pow(coll_vio, 3);
          Eigen::RowVector2d grad_p = params.w_obstacle * (-6) * std::pow(coll_vio, 2) * dist_vec;

          penalty += omg * h * cost_obs;
          grad_c.block(i * params.n_coeffs, 0, params.n_coeffs, params.dims) += omg * h * (B0 * grad_p);

          double alpha = (double)j / (double)kappa_i;
          double grad_T_obs_direct = alpha * grad_p.dot(vel);
          grad_T(i) += omg * (cost_obs / (double)kappa_i + h * grad_T_obs_direct);
        }
      }

      double v_pen = vel.squaredNorm() - params.v_max * params.v_max;
      if (v_pen > 0) {
        double cost_v = params.w_feasibility * std::pow(v_pen, 3);
        Eigen::RowVector2d grad_v = params.w_feasibility * 6 * std::pow(v_pen, 2) * vel;

        penalty += omg * h * cost_v;
        grad_c.block(i * params.n_coeffs, 0, params.n_coeffs, params.dims) += omg * h * (B1 * grad_v);

        double alpha = (double)j / (double)kappa_i;
        double grad_T_v_direct = alpha * grad_v.dot(acc);
        grad_T(i) += omg * (cost_v / (double)kappa_i + h * grad_T_v_direct);
      }

      double a_pen = acc.squaredNorm() - params.a_max * params.a_max;
      if (a_pen > 0) {
        double cost_a = params.w_feasibility * std::pow(a_pen, 3);
        Eigen::RowVector2d grad_a = params.w_feasibility * 6 * std::pow(a_pen, 2) * acc;

        penalty += omg * h * cost_a;
        grad_c.block(i * params.n_coeffs, 0, params.n_coeffs, params.dims) += omg * h * (B2 * grad_a);

        double alpha = (double)j / (double)kappa_i;
        double grad_T_a_direct = alpha * grad_a.dot(jer);
        grad_T(i) += omg * (cost_a / (double)kappa_i + h * grad_T_a_direct);
      }
    }
  }
}


void MochaOptimizer::backpropagateMochaGradient(const Eigen::VectorXd& T,
                                            const Eigen::MatrixXd& total_grad_c,
                                            const Eigen::VectorXd& total_grad_T_direct,
                                            const MochaParameters& params,
                                            const Eigen::MatrixXd& M_banded_LU,
                                            const Eigen::MatrixXd& coeffs,
                                            Eigen::MatrixXd& grad_q,
                                            Eigen::VectorXd& grad_T_total) {
  int n_segments = params.n_segments;
  int s = params.s;
  int n_coeffs = params.n_coeffs;
  int dims = params.dims;

  Eigen::MatrixXd G = solveBandedSystemAdj(M_banded_LU, total_grad_c, n_coeffs, n_coeffs);

  grad_q.resize(n_segments > 1 ? n_segments - 1 : 0, dims);
  for (int i = 0; i < n_segments - 1; ++i) {
    int r_waypoint = s + i * (2 * s) + (s - 1);
    grad_q.row(i) = G.row(r_waypoint);
  }

  Eigen::VectorXd grad_T_indirect = Eigen::VectorXd::Zero(n_segments);

  for (int i = 0; i < n_segments - 1; ++i) {
    double t_i = T(i);
    Eigen::MatrixXd dEi_dc = Eigen::MatrixXd::Zero(2 * s, dims);
    const auto& c_i = coeffs.block(i * n_coeffs, 0, n_coeffs, dims);

    for (int k = s; k < 2 * s - 1; ++k) {
      dEi_dc.row(k - s) = getPolyBasis(t_i, params.n_order, k + 1).transpose() * c_i;
    }
    dEi_dc.row(s - 1) = getPolyBasis(t_i, params.n_order, 1).transpose() * c_i;
    for (int k = 0; k < s; ++k) {
      dEi_dc.row(s + k) = getPolyBasis(t_i, params.n_order, k + 1).transpose() * c_i;
    }

    int r_block_start = s + i * (2 * s);
    grad_T_indirect(i) = (G.block(r_block_start, 0, 2 * s, dims).array() * dEi_dc.array()).sum();
  }

  if (n_segments > 0) {
    double t_M = T(n_segments - 1);
    Eigen::MatrixXd dEM_dc = Eigen::MatrixXd::Zero(s, dims);
    const auto& c_M = coeffs.block((n_segments - 1) * n_coeffs, 0, n_coeffs, dims);
    for (int k = 0; k < s; ++k) {
      dEM_dc.row(k) = getPolyBasis(t_M, params.n_order, k + 1).transpose() * c_M;
    }
    grad_T_indirect(n_segments - 1) = (G.bottomRows(s).array() * dEM_dc.array()).sum();
  }

  grad_T_total = total_grad_T_direct - grad_T_indirect;
}

} // namespace ghost_planner


