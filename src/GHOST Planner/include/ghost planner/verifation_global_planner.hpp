#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ghost_planner/msg/trajectory.hpp"
#include "ghost_planner/action/global_plan.hpp"
#include "ghost_planner/msg/committed_trajectory.hpp"

namespace ghost_planner {

class VerifationGlobalPlanner : public rclcpp::Node {
public:
  using GlobalPlan = ghost_planner::action::GlobalPlan;
  using GoalHandleGlobalPlan = rclcpp_action::ClientGoalHandle<GlobalPlan>;

  explicit VerifationGlobalPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Subscribers / publishers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sphere_pub_;
  rclcpp::Publisher<ghost_planner::msg::CommittedTrajectory>::SharedPtr committed_pub_;

  // Action client
  rclcpp_action::Client<GlobalPlan>::SharedPtr action_client_;

  // Current trajectory
  ghost_planner::msg::Trajectory current_traj_;
  bool has_traj_{false};
  rclcpp::Time traj_start_time_;
  rclcpp::Time planning_start_time_;

  // Timer for playback
  rclcpp::TimerBase::SharedPtr timer_;
  double playback_dt_{0.02};
  std::string global_frame_{"map"};
  
  // Initial position and color
  double start_x_{0.0};
  double start_y_{0.0};
  double color_r_{0.1};
  double color_g_{0.6};
  double color_b_{1.0};

  // Callbacks
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void timerCallback();

  // Helpers: evaluate polynomial position and velocity at time t
  Eigen::Vector2d evalPosition(double t) const;
  Eigen::Vector2d evalVelocity(double t) const;
};

} // namespace ghost_planner


