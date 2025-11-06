#include "ghost planner/verifation_global_planner.hpp"

#include <numeric>

namespace ghost_planner {

VerifationGlobalPlanner::VerifationGlobalPlanner(const rclcpp::NodeOptions & options)
  : Node("verifation_global_planner", options)
{
  this->declare_parameter("playback_dt", rclcpp::ParameterValue(0.02));
  this->declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
  this->declare_parameter("start.x", rclcpp::ParameterValue(0.0));
  this->declare_parameter("start.y", rclcpp::ParameterValue(0.0));
  this->declare_parameter("color.r", rclcpp::ParameterValue(0.1));
  this->declare_parameter("color.g", rclcpp::ParameterValue(0.6));
  this->declare_parameter("color.b", rclcpp::ParameterValue(1.0));
  
  this->get_parameter("playback_dt", playback_dt_);
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("start.x", start_x_);
  this->get_parameter("start.y", start_y_);
  this->get_parameter("color.r", color_r_);
  this->get_parameter("color.g", color_g_);
  this->get_parameter("color.b", color_b_);

  // per-robot action namespace: use node name as prefix, e.g., /robot_1/global_plan
  std::string action_ns = this->get_name() + std::string("/global_plan");
  action_client_ = rclcpp_action::create_client<GlobalPlan>(this, action_ns);
  {
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();
    committed_pub_ = this->create_publisher<ghost_planner::msg::CommittedTrajectory>("/team_committed_trajectory", qos);
  }

  // per-robot goal topic
  std::string goal_topic = std::string("/") + this->get_name() + std::string("/goal_pose");
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, 10, std::bind(&VerifationGlobalPlanner::goalCallback, this, std::placeholders::_1));

  // per-robot tracking sphere
  std::string sphere_topic = std::string("/") + this->get_name() + std::string("/tracking_sphere");
  sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(sphere_topic, 1);

  timer_ = this->create_wall_timer(std::chrono::duration<double>(playback_dt_), std::bind(&VerifationGlobalPlanner::timerCallback, this));
}

void VerifationGlobalPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "GlobalPlan action not available");
    return;
  }

  GlobalPlan::Goal goal;
  goal.goal = *msg;
  planning_start_time_ = this->now();

  auto send_options = rclcpp_action::Client<GlobalPlan>::SendGoalOptions();
  send_options.result_callback = [this](const GoalHandleGlobalPlan::WrappedResult & result) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Global plan failed");
      has_traj_ = false;
      return;
    }
    current_traj_ = result.result->trajectory;
    traj_start_time_ = this->now();
    has_traj_ = true;

    // Publish committed trajectory for peers
    ghost_planner::msg::CommittedTrajectory ct;
    ct.robot_id = this->get_name();
    ct.robot_radius = 0.3;
    // Use planning start time as committed trajectory epoch for better sync under latency
    ct.plan_start_time_sec = planning_start_time_.seconds();
    ct.dims = current_traj_.dims;
    ct.n_coeffs = current_traj_.n_coeffs;
    ct.n_segments = current_traj_.n_segments;
    ct.segment_durations = current_traj_.segment_durations;
    ct.poly_coeffs = current_traj_.poly_coeffs;
    committed_pub_->publish(ct);
  };

  action_client_->async_send_goal(goal, send_options);
}

Eigen::Vector2d VerifationGlobalPlanner::evalPosition(double t) const
{
  if (!has_traj_) return Eigen::Vector2d::Zero();
  const auto & T = current_traj_.segment_durations;
  const int dims = current_traj_.dims; // expect 2
  const int n_coeffs = current_traj_.n_coeffs; // expect 6
  double time_accum = 0.0;
  int seg = 0;
  for (; seg < (int)T.size(); ++seg) {
    if (t < time_accum + T[seg]) break;
    time_accum += T[seg];
  }
  
  // Handle out-of-range: clamp to last segment endpoint
  if (seg >= (int)T.size()) {
    seg = (int)T.size() - 1;
    // Recompute time_accum excluding the last segment
    time_accum = 0.0;
    for (int i = 0; i < seg; ++i) {
      time_accum += T[i];
    }
  }
  
  double ts = std::max(0.0, t - time_accum);
  ts = std::min(ts, T[seg]);

  // coeffs row-major: (n_coeffs*n_segments) x dims
  int row0 = seg * n_coeffs;
  auto coeff = [&](int r, int c) {
    size_t idx = static_cast<size_t>(r * dims + c);
    return current_traj_.poly_coeffs[idx];
  };
  double px = 0.0, py = 0.0, tp = 1.0;
  for (int i = 0; i < n_coeffs; ++i) {
    px += coeff(row0 + i, 0) * tp;
    py += coeff(row0 + i, 1) * tp;
    tp *= ts;
  }
  return Eigen::Vector2d(px, py);
}

Eigen::Vector2d VerifationGlobalPlanner::evalVelocity(double t) const
{
  if (!has_traj_) return Eigen::Vector2d::Zero();
  const auto & T = current_traj_.segment_durations;
  const int dims = current_traj_.dims; // expect 2
  const int n_coeffs = current_traj_.n_coeffs; // expect 6
  double time_accum = 0.0;
  int seg = 0;
  for (; seg < (int)T.size(); ++seg) {
    if (t < time_accum + T[seg]) break;
    time_accum += T[seg];
  }
  
  // Handle out-of-range: clamp to last segment endpoint
  if (seg >= (int)T.size()) {
    seg = (int)T.size() - 1;
    // Recompute time_accum excluding the last segment
    time_accum = 0.0;
    for (int i = 0; i < seg; ++i) {
      time_accum += T[i];
    }
  }
  
  double ts = std::max(0.0, t - time_accum);
  ts = std::min(ts, T[seg]);

  int row0 = seg * n_coeffs;
  auto coeff = [&](int r, int c) {
    size_t idx = static_cast<size_t>(r * dims + c);
    return current_traj_.poly_coeffs[idx];
  };
  double vx = 0.0, vy = 0.0;
  double tp = 1.0;
  for (int i = 1; i < n_coeffs; ++i) {
    vx += i * coeff(row0 + i, 0) * tp;
    vy += i * coeff(row0 + i, 1) * tp;
    tp *= ts;
  }
  return Eigen::Vector2d(vx, vy);
}

void VerifationGlobalPlanner::timerCallback()
{
  Eigen::Vector2d p;
  
  if (!has_traj_) {
    // No trajectory yet, show sphere at initial position
    p = Eigen::Vector2d(start_x_, start_y_);
  } else {
    // Follow the trajectory
    double t_now = (this->now() - traj_start_time_).seconds();
    double total_T = current_traj_.total_duration;
    if (t_now > total_T) t_now = total_T;
    p = evalPosition(t_now);
  }

  visualization_msgs::msg::Marker m;
  m.header.frame_id = global_frame_;
  m.header.stamp = this->now();
  m.ns = "verifation_global_planner";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = p.x();
  m.pose.position.y = p.y();
  m.pose.position.z = 0.3;
  m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 0.25;
  m.color.a = 0.95; 
  m.color.r = static_cast<float>(color_r_); 
  m.color.g = static_cast<float>(color_g_); 
  m.color.b = static_cast<float>(color_b_);
  sphere_pub_->publish(m);
}

} // namespace ghost_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ghost_planner::VerifationGlobalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


