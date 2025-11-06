#pragma once

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace ghost_planner {

struct CircleObstacle {
  Eigen::Vector2d center;
  double radius;
};

struct Graph {
  std::vector<Eigen::Vector2d> nodes;
  std::vector<std::vector<std::pair<int,double>>> adj;
};

struct GeometricPath {
  std::vector<Eigen::Vector2d> points;
  std::string homotopy_signature;
  double length = 0.0;
};

class CgalHomotopyPlanner {
public:
  struct Options {
    size_t max_number_of_paths = 6;
    double safety_margin = 0.10;
    bool enable_bezier_smoothing = true;
    int bezier_samples_per_segment = 8;
    double smoothing_collision_step = 0.05;
    double bbox_expansion_margin = 5.0;
  };

  CgalHomotopyPlanner() = default;

  std::vector<GeometricPath> generatePaths(
      const Eigen::Vector2d &start,
      const Eigen::Vector2d &goal,
      const std::vector<CircleObstacle> &obstacles,
      const Options &options) const;

  static nav_msgs::msg::Path toRosPath(
      const GeometricPath &path,
      const std::string &frame_id,
      int32_t stamp_sec = 0,
      uint32_t stamp_nanosec = 0);

  std::vector<GeometricPath> generatePaths(
      const Eigen::Vector2d &start,
      const Eigen::Vector2d &goal,
      const std::vector<CircleObstacle> &obstacles) const;

  struct SkeletonEdge { Eigen::Vector2d a; Eigen::Vector2d b; };
  std::vector<SkeletonEdge> buildSkeletonEdges(
      const std::vector<CircleObstacle> &obstacles,
      const Options &options,
      const Eigen::Vector2d &start = Eigen::Vector2d::Zero(),
      const Eigen::Vector2d &goal  = Eigen::Vector2d::Zero()) const;

private:
  struct ApolloniusEdge {
    Eigen::Vector2d source, target;
    double length;
  };

  struct ApolloniusGraph {
    std::vector<Eigen::Vector2d> vertices;
    std::vector<ApolloniusEdge> edges;
    std::vector<std::vector<std::pair<int, double>>> adjacency;
  };

  ApolloniusGraph buildObstacleAvoidingSkeleton(
      const std::vector<CircleObstacle> &obstacles,
      const Options &options,
      const Eigen::Vector2d &start = Eigen::Vector2d::Zero(),
      const Eigen::Vector2d &goal = Eigen::Vector2d::Zero()) const;

  std::vector<Eigen::Vector2d> smoothPathWithBezier(
      const std::vector<Eigen::Vector2d> &path,
      const std::vector<CircleObstacle> &obstacles,
      const Options &options) const;

  bool validateSmoothedPath(
      const std::vector<Eigen::Vector2d> &path,
      const std::vector<CircleObstacle> &obstacles,
      double safety_margin,
      double step_size) const;

  int connectStartGoalRobustly(
      const Eigen::Vector2d &point,
      int point_id,
      Graph &graph,
      const std::vector<CircleObstacle> &obstacles,
      double safety_margin) const;
      
  std::vector<Eigen::Vector2d> computeTangentPoints(
      const Eigen::Vector2d &point,
      const CircleObstacle &obstacle,
      double safety_margin) const;
};

}
