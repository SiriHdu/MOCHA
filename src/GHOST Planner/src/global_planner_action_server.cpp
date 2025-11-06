#include "ghost planner/global_planner_action_server.hpp"

#include <limits>
#include <chrono>
#include <future>
#include <unordered_set>

using namespace std::chrono_literals;

namespace ghost_planner {

GlobalPlannerActionServer::GlobalPlannerActionServer(const rclcpp::NodeOptions & options)
  : Node("global_planner", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Global Planner Action Server...");


  this->declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
  this->declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);


  this->declare_parameter("self_robot_id", rclcpp::ParameterValue(std::string("robot_1")));
  this->declare_parameter("peer_ttl_sec", rclcpp::ParameterValue(10.0));
  this->declare_parameter("all_robot_ids", rclcpp::ParameterValue(std::vector<std::string>{}));
  this->declare_parameter("idle_robot_radius", rclcpp::ParameterValue(0.3));
  this->get_parameter("self_robot_id", self_robot_id_);
  this->get_parameter("peer_ttl_sec", peer_ttl_sec_);
  this->get_parameter("idle_robot_radius", idle_robot_radius_);


  this->declare_parameter(" mocha.v_max", rclcpp::ParameterValue(2.5));
  this->declare_parameter(" mocha.a_max", rclcpp::ParameterValue(5.0));
  this->declare_parameter(" mocha.drone_radius", rclcpp::ParameterValue(0.3));
  this->declare_parameter(" mocha.w_energy", rclcpp::ParameterValue(1.0));
  this->declare_parameter(" mocha.w_time", rclcpp::ParameterValue(1.0));
  this->declare_parameter(" mocha.w_feasibility", rclcpp::ParameterValue(100.0));
  this->declare_parameter(" mocha.w_obstacle", rclcpp::ParameterValue(100.0));
  this->declare_parameter(" mocha.kappa", rclcpp::ParameterValue(10));
  this->declare_parameter(" mocha.w_peer", rclcpp::ParameterValue(200.0));
  this->declare_parameter(" mocha.peer_safety_margin", rclcpp::ParameterValue(0.10));

  this->get_parameter(" mocha.v_max", mocha_params_.v_max);
  this->get_parameter(" mocha.a_max", mocha_params_.a_max);
  this->get_parameter(" mocha.drone_radius", mocha_params_.drone_radius);
  this->get_parameter(" mocha.w_energy", mocha_params_.w_energy);
  this->get_parameter(" mocha.w_time", mocha_params_.w_time);
  this->get_parameter(" mocha.w_feasibility", mocha_params_.w_feasibility);
  this->get_parameter(" mocha.w_obstacle", mocha_params_.w_obstacle);
  this->get_parameter(" mocha.kappa", mocha_params_.kappa);
  this->get_parameter(" mocha.w_peer", mocha_params_.w_peer);
  this->get_parameter(" mocha.peer_safety_margin", mocha_params_.peer_safety_margin);


  int max_paths = 6;
  this->declare_parameter("cgal.max_paths", rclcpp::ParameterValue(6));
  this->declare_parameter("cgal.safety_margin", rclcpp::ParameterValue(0.1));
  this->declare_parameter("cgal.enable_bezier_smoothing", rclcpp::ParameterValue(false));
  this->declare_parameter("cgal.bezier_samples_per_segment", rclcpp::ParameterValue(8));
  this->declare_parameter("cgal.smoothing_collision_step", rclcpp::ParameterValue(0.05));
  this->declare_parameter("cgal.bbox_expansion_margin", rclcpp::ParameterValue(5.0));
  this->declare_parameter("cgal.show_skeleton", rclcpp::ParameterValue(false));

  this->get_parameter("cgal.max_paths", max_paths);
  cgal_opts_.max_number_of_paths = static_cast<size_t>(std::max(1, max_paths));
  this->get_parameter("cgal.safety_margin", cgal_opts_.safety_margin);
  this->get_parameter("cgal.enable_bezier_smoothing", cgal_opts_.enable_bezier_smoothing);
  this->get_parameter("cgal.bezier_samples_per_segment", cgal_opts_.bezier_samples_per_segment);
  this->get_parameter("cgal.smoothing_collision_step", cgal_opts_.smoothing_collision_step);
  this->get_parameter("cgal.bbox_expansion_margin", cgal_opts_.bbox_expansion_margin);
  this->get_parameter("cgal.show_skeleton", show_skeleton_);


  this->declare_parameter("trajectory.resample_step_m", rclcpp::ParameterValue(3.0));
  this->declare_parameter("trajectory.viz_sample_dt", rclcpp::ParameterValue(0.02));
  this->declare_parameter("trajectory.show_initial_waypoints", rclcpp::ParameterValue(false));
  this->declare_parameter("trajectory.show_cgal_raw", rclcpp::ParameterValue(false));
  this->get_parameter("trajectory.resample_step_m", resample_step_m_);
  this->get_parameter("trajectory.viz_sample_dt", viz_sample_dt_);
  this->get_parameter("trajectory.show_initial_waypoints", show_initial_waypoints_);
  this->get_parameter("trajectory.show_cgal_raw", show_cgal_raw_);


  this->declare_parameter("ref.distance_ratio", rclcpp::ParameterValue(0.5));
  this->declare_parameter("ref.weight_decay_distance", rclcpp::ParameterValue(100.0));
  this->declare_parameter("ref.use_obstacle_side_selection", rclcpp::ParameterValue(true));
  this->declare_parameter("ref.manual_side", rclcpp::ParameterValue(std::string("left")));
  this->get_parameter("ref.distance_ratio", ref_distance_ratio_);
  this->get_parameter("ref.weight_decay_distance", ref_weight_decay_distance_);
  this->get_parameter("ref.use_obstacle_side_selection", ref_use_obstacle_side_selection_);
  this->get_parameter("ref.manual_side", ref_manual_side_);


  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/obstacles", rclcpp::QoS(1).transient_local(),
    std::bind(&GlobalPlannerActionServer::obstaclesCallback, this, std::placeholders::_1));


  {
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();
    peers_sub_ = this->create_subscription<ghost_planner::msg::CommittedTrajectory>(
      "/team_committed_trajectory", qos,
      std::bind(&GlobalPlannerActionServer::peerCommittedCallback, this, std::placeholders::_1));
  }

  rclcpp::QoS viz_qos(1);
  viz_qos.transient_local();
  viz_qos.reliable();


  std::string ns = this->get_namespace();
  std::string best_topic = ns.empty() || ns == "/" ? std::string("/best_path") : ns + std::string("/best_path");
  best_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(best_topic, viz_qos);
  candidates_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cgal_candidate_paths_markers", viz_qos);
  cgal_raw_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cgal_raw_paths", viz_qos);
  optimized_paths_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("optimized_paths_markers", viz_qos);
  initial_waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("initial_waypoints_markers", viz_qos);
  skeleton_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cgal_skeleton", viz_qos);
  global_traj_pub_ = this->create_publisher<ghost_planner::msg::Trajectory>("global_trajectory", rclcpp::QoS(1).transient_local());


  action_server_ = rclcpp_action::create_server<GlobalPlan>(
    this,
    "global_plan",
    std::bind(&GlobalPlannerActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlannerActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&GlobalPlannerActionServer::handleAccepted, this, std::placeholders::_1));
}

void GlobalPlannerActionServer::obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  obstacles_eig_.clear();
  obstacles_eig_.reserve(msg->markers.size());
  for (const auto& marker : msg->markers) {
    obstacles_eig_.emplace_back(marker.pose.position.x,
                                marker.pose.position.y,
                                marker.scale.x / 2.0);
  }
  obstacles_received_ = true;
}

void GlobalPlannerActionServer::peerCommittedCallback(const ghost_planner::msg::CommittedTrajectory::SharedPtr msg)
{
  if (msg->robot_id == self_robot_id_) return;
  PeerCache cache;
  cache.robot_id = msg->robot_id;
  cache.radius = msg->robot_radius;
  cache.plan_start_sec = msg->plan_start_time_sec;
  cache.dims = msg->dims;
  cache.n_coeffs = msg->n_coeffs;
  cache.n_segments = msg->n_segments;
  cache.last_seen = this->now();
  if (cache.n_segments > 0 && (int)msg->segment_durations.size() == cache.n_segments) {
    cache.segment_T = Eigen::Map<const Eigen::VectorXd>(msg->segment_durations.data(), cache.n_segments);
  }
  if (!msg->poly_coeffs.empty() && cache.n_segments > 0 && cache.n_coeffs > 0 && cache.dims > 0) {
    int rows = cache.n_coeffs * cache.n_segments;
    int cols = cache.dims;
    if ((int)msg->poly_coeffs.size() == rows * cols) {
      cache.coeffs = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->poly_coeffs.data(), rows, cols);
      cache.has_poly = cache.segment_T.size() == cache.n_segments;
    }
  }
  {
    std::lock_guard<std::mutex> lk(peers_mutex_);
    peers_[cache.robot_id] = std::move(cache);
  }
}

rclcpp_action::GoalResponse GlobalPlannerActionServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GlobalPlan::Goal> /*goal*/)
{
  if (!obstacles_received_) {
    RCLCPP_WARN(this->get_logger(), "No obstacles received yet, but accepting goal.");
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlannerActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlannerActionServer::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&GlobalPlannerActionServer::execute, this, goal_handle)}.detach();
}

void GlobalPlannerActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GlobalPlan::Feedback>();
  auto result = std::make_shared<GlobalPlan::Result>();


  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero, std::chrono::seconds(1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
    result->success = false;
    result->message = std::string("TF error: ") + ex.what();
    goal_handle->abort(result);
    return;
  }

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = transform.transform.translation.x;
  start_pose.position.y = transform.transform.translation.y;

  Eigen::Vector2d start_point(start_pose.position.x, start_pose.position.y);
  Eigen::Vector2d goal_point(goal->goal.pose.position.x, goal->goal.pose.position.y);


  std::vector<CircleObstacle> circles;
  circles.reserve(obstacles_eig_.size());
  for (const auto& o : obstacles_eig_) {
    circles.push_back(CircleObstacle{ o.head<2>(), o.z() });
  }

  feedback->status = "Generating CGAL paths";
  feedback->progress = 0.1f;
  goal_handle->publish_feedback(feedback);

  CgalHomotopyPlanner planner;
  auto paths = planner.generatePaths(start_point, goal_point, circles, cgal_opts_);


  if (show_skeleton_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = this->now();
    m.ns = "cgal_skeleton";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.color.a = 0.9; m.color.r = 0.1f; m.color.g = 0.4f; m.color.b = 1.0f;
    auto edges = planner.buildSkeletonEdges(circles, cgal_opts_, start_point, goal_point);
    m.points.clear(); m.points.reserve(edges.size() * 2);
    for (const auto &e : edges) {
      geometry_msgs::msg::Point pa, pb; pa.x = e.a.x(); pa.y = e.a.y(); pa.z = 0.05; pb.x = e.b.x(); pb.y = e.b.y(); pb.z = 0.05; m.points.push_back(pa); m.points.push_back(pb);
    }
    skeleton_pub_->publish(m);
  }
  if (paths.empty()) {
    result->success = false;
    result->message = "CGAL generated no paths";
    goal_handle->abort(result);
    return;
  }


  visualization_msgs::msg::MarkerArray clear_ms;
  candidates_markers_pub_->publish(clear_ms);
  cgal_raw_paths_pub_->publish(clear_ms);
  optimized_paths_markers_pub_->publish(clear_ms);

  if (show_cgal_raw_) {
    visualization_msgs::msg::MarkerArray raw_ms;
    int id = 0;
    for (size_t i = 0; i < paths.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = global_frame_;
      m.header.stamp = this->now();
      m.ns = std::string("cgal_raw_") + std::to_string(i);
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.05;
      m.color.a = 0.9; m.color.r = 1.0f; m.color.g = 0.6f; m.color.b = 0.2f;
      for (const auto& p : paths[i].points) {
        geometry_msgs::msg::Point pt; pt.x = p.x(); pt.y = p.y(); pt.z = 0.1;
        m.points.push_back(pt);
      }
      raw_ms.markers.push_back(m);
    }
    cgal_raw_paths_pub_->publish(raw_ms);
  }

  feedback->status = "Optimizing candidates (MOCHA)";
  feedback->progress = 0.3f;




  current_plan_start_sec_ = this->now().seconds();
  struct Task {
    int idx;
    MochaParameters P;
  };
  std::vector<Task> tasks;
  tasks.reserve(paths.size());

  int path_idx = 0;
  for (const auto& gp : paths) {
    const double step = std::max(0.1, resample_step_m_);
    auto resampled = resamplePolyline(gp.points, step);
    if (resampled.size() < 2) { ++path_idx; continue; }

    if (show_initial_waypoints_) {
      visualization_msgs::msg::MarkerArray ms;
      visualization_msgs::msg::Marker clear;
      clear.header.frame_id = global_frame_;
      clear.header.stamp = this->now();
      clear.ns = std::string("initial_wp_") + std::to_string(path_idx);
      clear.id = 0;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      ms.markers.push_back(clear);
      initial_waypoints_pub_->publish(ms);

      ms.markers.clear();
      int id = 0;
      for (const auto& p : resampled) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = this->now();
        m.ns = clear.ns;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = p.x();
        m.pose.position.y = p.y();
        m.pose.position.z = 0.3;
        m.scale.x = 0.22; m.scale.y = 0.22; m.scale.z = 0.22;
        m.color.a = 0.95; m.color.r = 1.0f; m.color.g = 0.1f; m.color.b = 0.1f;
        ms.markers.push_back(m);
      }
      initial_waypoints_pub_->publish(ms);
    }

    MochaParameters P = mocha_params_;
    P.n_segments = static_cast<int>(resampled.size()) - 1;
    P.prey_points = resampled;
    P.start_waypoint = resampled.front();
    P.end_waypoint = resampled.back();
    P.obstacles = obstacles_eig_;
    

    std::unordered_set<std::string> robots_with_traj;
    {
      std::lock_guard<std::mutex> lk(peers_mutex_);
      for (const auto& [rid, pc] : peers_) {
        if (pc.has_poly && pc.n_segments > 0) {
          robots_with_traj.insert(rid);
        }
      }
    }
    
    for (const auto& robot_id : all_robot_ids_) {
      if (robot_id == self_robot_id_) continue;
      if (robots_with_traj.count(robot_id) > 0) continue;
      

      std::string peer_frame = robot_id + "/base_link";
      try {
        auto transform = tf_buffer_->lookupTransform(
          global_frame_, peer_frame, tf2::TimePointZero, std::chrono::milliseconds(100));
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

        P.obstacles.emplace_back(x, y, idle_robot_radius_ + 0.3);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Cannot get TF for idle robot %s: %s", robot_id.c_str(), ex.what());
      }
    }
    
    P.ref_point = selectReferencePointLikeVMC(P.prey_points, P.obstacles);
    P.initial_v.assign(std::max(0, P.n_segments - 1), 1.0);
    {
      std::lock_guard<std::mutex> lk(peers_mutex_);
      P.peer_trajs.clear();
      const double now_sec = current_plan_start_sec_;
      for (const auto& [rid, pc] : peers_) {
        if (rid == self_robot_id_) continue;
        if ((this->now() - pc.last_seen).seconds() > peer_ttl_sec_) continue;
        if (!pc.has_poly || pc.n_segments == 0) continue;
        MochaParameters::PeerSampledTraj pt;
        pt.radius = pc.radius;
        pt.plan_start_sec = pc.plan_start_sec;
        pt.dims = pc.dims;
        pt.n_coeffs = pc.n_coeffs;
        pt.n_segments = pc.n_segments;
        pt.coeffs = pc.coeffs;
        pt.segment_T = pc.segment_T;
        pt.has_poly = pc.has_poly;
        P.peer_trajs.push_back(std::move(pt));
      }
      P.self_plan_start_sec = now_sec;
    }
    tasks.push_back(Task{path_idx, P});
    ++path_idx;
  }


  struct Result { bool ok; MochaTrajectory traj; MochaParameters P; int idx; };
  std::vector<std::future<Result>> futures;
  futures.reserve(tasks.size());
  auto wall_t0 = std::chrono::high_resolution_clock::now();
  
  for (const auto& t : tasks) {
    futures.emplace_back(std::async(std::launch::async, [this, t]() -> Result {
      MochaTrajectory traj;
      bool ok = MochaOptimizer::optimize(t.P, traj, this->get_logger());
      return Result{ok && traj.isValid(), std::move(traj), t.P, t.idx};
    }));
  }

  double best_cost = std::numeric_limits<double>::infinity();
  nav_msgs::msg::Path best_nav_path;
  bool best_found = false;
  Eigen::MatrixXd best_coeffs;
  Eigen::VectorXd best_T;

  for (auto& fut : futures) {
    Result r = fut.get();
    if (!r.ok) {
      continue;
    }
    if (r.traj.final_cost < best_cost) {
      best_cost = r.traj.final_cost;
      best_coeffs = r.traj.coeffs;
      best_T = r.traj.T;
      best_nav_path = sampleTrajectory(best_coeffs, best_T, r.P);
      best_nav_path.header.stamp = this->now();
      best_nav_path.header.frame_id = global_frame_;
      best_found = true;
    }
  }
  auto wall_t1 = std::chrono::high_resolution_clock::now();
  RCLCPP_INFO(this->get_logger(), "Optimized %zu candidates in %.3f s. Best cost: %.6f", 
              tasks.size(), std::chrono::duration<double>(wall_t1 - wall_t0).count(), 
              best_cost);

  if (!best_found) {
    result->success = false;
    result->message = "No valid MOCHA trajectory among candidates";
    goal_handle->abort(result);
    return;
  }

  best_path_pub_->publish(best_nav_path);

  visualization_msgs::msg::MarkerArray opt_ms;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_;
  m.header.stamp = this->now();
  m.ns = std::string("optimized_best");
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.08;
  m.color.a = 0.95; m.color.r = 0.1f; m.color.g = 0.8f; m.color.b = 0.2f;
  for (const auto& ps : best_nav_path.poses) {
    geometry_msgs::msg::Point pt; pt.x = ps.pose.position.x; pt.y = ps.pose.position.y; pt.z = 0.2;
    m.points.push_back(pt);
  }
  opt_ms.markers.push_back(m);
  optimized_paths_markers_pub_->publish(opt_ms);

  if (best_T.size() > 0 && best_coeffs.size() > 0) {
    ghost_planner::msg::Trajectory msg;
    msg.frame_id = global_frame_;
    msg.traj_id = std::string("global_best");
    msg.is_local = false;
    msg.is_global = true;
    msg.dims = mocha_params_.dims;
    msg.n_coeffs = mocha_params_.n_coeffs;
    msg.n_segments = static_cast<int32_t>(best_T.size());
    double total_T = 0.0;
    msg.segment_durations.clear();
    msg.segment_durations.reserve(best_T.size());
    for (int i = 0; i < best_T.size(); ++i) { msg.segment_durations.push_back(best_T(i)); total_T += best_T(i);} 
    msg.total_duration = total_T;
    msg.plan_start_time_sec = current_plan_start_sec_;
    msg.poly_coeffs.clear();
    msg.poly_coeffs.reserve(static_cast<size_t>(best_coeffs.rows() * best_coeffs.cols()));
    for (int r = 0; r < best_coeffs.rows(); ++r) {
      for (int c = 0; c < best_coeffs.cols(); ++c) {
        msg.poly_coeffs.push_back(best_coeffs(r, c));
      }
    }
    global_traj_pub_->publish(msg);
    result->trajectory = msg;
  }

  result->best_path = best_nav_path;
  result->success = true;
  result->message = "OK";
  goal_handle->succeed(result);
}

std::vector<Eigen::Vector2d> GlobalPlannerActionServer::resamplePolyline(const std::vector<Eigen::Vector2d>& polyline, double step)
{
  if (polyline.size() <= 1) return polyline;
  std::vector<Eigen::Vector2d> out; out.reserve(polyline.size());
  out.push_back(polyline.front());
  double accum = 0.0;
  for (size_t i = 1; i < polyline.size(); ++i) {
    Eigen::Vector2d a = polyline[i-1];
    Eigen::Vector2d b = polyline[i];
    double seg_len = (b - a).norm();
    if (seg_len < 1e-9) continue;
    Eigen::Vector2d dir = (b - a) / seg_len;
    while (accum + seg_len >= step) {
      double t = step - accum;
      out.push_back(a + t * dir);
      a += t * dir;
      seg_len -= t;
      accum = 0.0;
    }
    accum += seg_len;
  }
  if ((out.back() - polyline.back()).norm() > 1e-6) out.push_back(polyline.back());
  return out;
}

Eigen::Vector2d GlobalPlannerActionServer::selectReferencePointLikeVMC(const std::vector<Eigen::Vector2d>& prey_path,
                                                                       const std::vector<Eigen::Vector3d>& obstacles)
{
  if (prey_path.size() < 2) return prey_path.empty() ? Eigen::Vector2d::Zero() : prey_path.front();
  const auto& p_start = prey_path.front();
  const auto& p_end = prey_path.back();
  Eigen::Vector2d path_vec = p_end - p_start;
  Eigen::Vector2d path_dir = path_vec.normalized();

  bool select_left = true;
  if (ref_use_obstacle_side_selection_) {
    double omega_left = 0.0, omega_right = 0.0;
    for (const auto& obs : obstacles) {
      Eigen::Vector2d obs_center = obs.head<2>();
      double obs_radius = obs.z();
      double t = path_dir.dot(obs_center - p_start);
      Eigen::Vector2d closest_point_on_path;
      if (t < 0) closest_point_on_path = p_start;
      else if (t > path_vec.norm()) closest_point_on_path = p_end;
      else closest_point_on_path = p_start + t * path_dir;
      double di = (obs_center - closest_point_on_path).norm();
      double Si = M_PI * obs_radius * obs_radius;
      double coeff = std::exp(-di / std::max(1e-6, ref_weight_decay_distance_)) * Si;
      double cross_product = path_vec.x() * (obs_center.y() - p_start.y()) - path_vec.y() * (obs_center.x() - p_start.x());
      if (cross_product > 0) omega_left += coeff; else omega_right += coeff;
    }
    select_left = (omega_left <= omega_right);
  } else {
    select_left = (ref_manual_side_ != "right");
  }

  double path_len = path_vec.norm();
  double distance_along_perp = std::max(0.0, ref_distance_ratio_) * path_len;
  Eigen::Vector2d mid_path_point = p_start + 0.5 * path_vec;
  Eigen::Vector2d perp_vec(-path_vec.y(), path_vec.x());
  if (perp_vec.norm() < 1e-9) return mid_path_point;
  perp_vec.normalize();
  if (!select_left) perp_vec = -perp_vec;
  return mid_path_point + distance_along_perp * perp_vec;
}

nav_msgs::msg::Path GlobalPlannerActionServer::sampleTrajectory(const Eigen::MatrixXd& coeffs,
                                                                const Eigen::VectorXd& T,
                                                                const MochaParameters& params)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i < params.n_segments; ++i) {
    Eigen::MatrixXd c_i = coeffs.block(i * params.n_coeffs, 0, params.n_coeffs, params.dims);
    double segment_duration = T(i);
    for (double t = 0.0; t < segment_duration; t += std::max(1e-3, viz_sample_dt_)) {
      Eigen::VectorXd B0 = MochaOptimizer::getPolyBasis(t, params.n_order, 0);
      Eigen::RowVector2d pos = B0.transpose() * c_i;
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = pos.x();
      ps.pose.position.y = pos.y();
      path.poses.push_back(ps);
    }
  }
  Eigen::MatrixXd c_final = coeffs.block((params.n_segments - 1) * params.n_coeffs, 0, params.n_coeffs, params.dims);
  Eigen::VectorXd B0_final = MochaOptimizer::getPolyBasis(T(params.n_segments - 1), params.n_order, 0);
  Eigen::RowVector2d pos_final = B0_final.transpose() * c_final;
  geometry_msgs::msg::PoseStamped pose_final;
  pose_final.pose.position.x = pos_final.x();
  pose_final.pose.position.y = pos_final.y();
  path.poses.push_back(pose_final);
  return path;
}

// evaluateCost removed; using traj.final_cost directly

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ghost_planner::GlobalPlannerActionServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


