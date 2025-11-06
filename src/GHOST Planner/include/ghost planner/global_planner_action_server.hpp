#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <thread>
#include <unordered_map>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"

#include "ghost planner/cgal_homotopy_planner.hpp"
#include "ghost planner/mocha_optimizer.hpp"

#include "ghost_planner/action/global_plan.hpp"
#include "ghost_planner/msg/trajectory.hpp"
#include "ghost_planner/msg/committed_trajectory.hpp"

namespace ghost_planner {

class GlobalPlannerActionServer : public rclcpp::Node {
public:
  using GlobalPlan = ghost_planner::action::GlobalPlan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GlobalPlan>;

  explicit GlobalPlannerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::string global_frame_{"map"};
  std::string robot_base_frame_{"base_link"};

  MochaParameters mocha_params_{};
  double resample_step_m_{1.0};
  double viz_sample_dt_{0.02};
  bool show_initial_waypoints_{false};
  bool show_cgal_raw_{false};

  double ref_distance_ratio_{0.5};
  double ref_weight_decay_distance_{100.0};
  bool ref_use_obstacle_side_selection_{true};
  std::string ref_manual_side_{"left"};

  CgalHomotopyPlanner::Options cgal_opts_{};
  bool show_skeleton_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<ghost_planner::msg::CommittedTrajectory>::SharedPtr peers_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr best_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cgal_raw_paths_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr optimized_paths_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr initial_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr skeleton_pub_;
  rclcpp::Publisher<ghost_planner::msg::Trajectory>::SharedPtr global_traj_pub_;

  rclcpp_action::Server<GlobalPlan>::SharedPtr action_server_;

  std::vector<Eigen::Vector3d> obstacles_eig_;
  bool obstacles_received_{false};
  double current_plan_start_sec_{0.0};

  struct PeerCache {
    std::string robot_id;
    double radius{0.2};
    double plan_start_sec{0.0};
    int dims{2};
    int n_coeffs{6};
    int n_segments{0};
    Eigen::MatrixXd coeffs;
    Eigen::VectorXd segment_T;
    bool has_poly{false};
    rclcpp::Time last_seen;
  };
  std::mutex peers_mutex_;
  std::unordered_map<std::string, PeerCache> peers_;
  double peer_ttl_sec_{3.0};
  std::string self_robot_id_{"robot_1"};
  std::vector<std::string> all_robot_ids_;
  double idle_robot_radius_{0.3};

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid,
                                         std::shared_ptr<const GlobalPlan::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);

  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  std::vector<Eigen::Vector2d> resamplePolyline(const std::vector<Eigen::Vector2d>& polyline, double step);
  Eigen::Vector2d selectReferencePointLikeVMC(const std::vector<Eigen::Vector2d>& prey_path,
                                              const std::vector<Eigen::Vector3d>& obstacles);
  nav_msgs::msg::Path sampleTrajectory(const Eigen::MatrixXd& coeffs,
                                       const Eigen::VectorXd& T,
                                       const MochaParameters& params);

  void obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void peerCommittedCallback(const ghost_planner::msg::CommittedTrajectory::SharedPtr msg);
};

} // namespace ghost_planner
