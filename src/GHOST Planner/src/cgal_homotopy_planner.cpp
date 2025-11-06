#include "ghost planner/cgal_homotopy_planner.hpp"

#include <rclcpp/rclcpp.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Apollonius_graph_2.h>
#include <CGAL/Apollonius_graph_adaptation_traits_2.h>
#include <CGAL/Apollonius_graph_traits_2.h>
#include <CGAL/Apollonius_site_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Line_2.h>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <sstream>

namespace ghost_planner {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Site = CGAL::Apollonius_site_2<K>;
using Traits = CGAL::Apollonius_graph_traits_2<K>;
using AG2 = CGAL::Apollonius_graph_2<Traits>;
using AT = CGAL::Apollonius_graph_adaptation_traits_2<AG2>;
using VD = CGAL::Voronoi_diagram_2<AG2, AT>;

static inline double segPointDistance(const Eigen::Vector2d &a,
                                      const Eigen::Vector2d &b,
                                      const Eigen::Vector2d &p) {
  const Eigen::Vector2d ab = b - a;
  const double ab2 = ab.squaredNorm();
  if (ab2 <= 1e-18) return (p - a).norm();
  double t = (p - a).dot(ab) / ab2;
  t = std::max(0.0, std::min(1.0, t));
  const Eigen::Vector2d proj = a + t * ab;
  return (p - proj).norm();
}

static inline bool segmentFreeOfCircles(const Eigen::Vector2d &a,
                                        const Eigen::Vector2d &b,
                                        const std::vector<CircleObstacle> &obs,
                                        double safety) {
  for (const auto &o : obs) {
    if (segPointDistance(a, b, o.center) < o.radius + safety - 1e-9) return false;
  }
  return true;
}

static inline bool pointFreeOfCircles(const Eigen::Vector2d &p,
                                      const std::vector<CircleObstacle> &obs,
                                      double safety) {
  for (const auto &o : obs) {
    if ((p - o.center).norm() < o.radius + safety - 1e-9) return false;
  }
  return true;
}

static std::string computeHomotopySignature(
    const std::vector<Eigen::Vector2d> &pts,
    const std::vector<CircleObstacle> &obstacles) {
  auto clampAngle = [](double a){ while (a <= -M_PI) a += 2*M_PI; while (a > M_PI) a -= 2*M_PI; return a; };
  std::ostringstream oss; bool first = true;
  for (size_t k = 0; k < obstacles.size(); ++k) {
    const auto &o = obstacles[k];
    double winding = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
      Eigen::Vector2d a = pts[i-1] - o.center;
      Eigen::Vector2d b = pts[i]   - o.center;
      double ang_a = std::atan2(a.y(), a.x());
      double ang_b = std::atan2(b.y(), b.x());
      winding += clampAngle(ang_b - ang_a);
    }
    int w = (int)std::llround(winding / (2.0 * M_PI));
    if (w != 0) { if (!first) oss << ";"; first = false; oss << "w[" << k << "]=" << w; }
  }
  std::string s = oss.str();
  if (s.empty()) s = "w=0";
  return s;
}


static inline std::pair<long long,long long> key2d(const Eigen::Vector2d &p, double res = 1e-6) {
  return { (long long)std::llround(p.x()/res), (long long)std::llround(p.y()/res) };
}

static int getOrAddNode(const Eigen::Vector2d &p,
                        Graph &g,
                        std::unordered_map<long long, int> &hashmap,
                        double res = 1e-6) {
  auto q = key2d(p, res);
  long long h = (q.first<<32) ^ (q.second & 0xffffffff);
  auto it = hashmap.find(h);
  if (it != hashmap.end()) return it->second;
  int id = (int)g.nodes.size();
  g.nodes.push_back(p);
  g.adj.emplace_back();
  hashmap[h] = id;
  return id;
}

static std::vector<int> dijkstra(const Graph &g, int s, int t) {
  const int n = (int)g.nodes.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> prev(n, -1);
  using P = std::pair<double,int>;
  std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
  dist[s] = 0.0; pq.push({0.0, s});
  while (!pq.empty()) {
    auto top = pq.top(); pq.pop();
    double d = top.first; int u = top.second;
    if (d != dist[u]) continue;
    if (u == t) break;
    for (const auto &e : g.adj[u]) {
      int v = e.first; double w = e.second;
      if (dist[v] > d + w) { dist[v] = d + w; prev[v] = u; pq.push({dist[v], v}); }
    }
  }
  std::vector<int> path;
  if (prev[t] == -1 && s != t) return path;
  for (int v = t; v != -1; v = prev[v]) path.push_back(v);
  std::reverse(path.begin(), path.end());
  return path;
}
nav_msgs::msg::Path CgalHomotopyPlanner::toRosPath(
    const GeometricPath &path,
    const std::string &frame_id,
    int32_t stamp_sec,
    uint32_t stamp_nanosec) {
  nav_msgs::msg::Path out;
  out.header.frame_id = frame_id;
  out.header.stamp.sec = stamp_sec;
  out.header.stamp.nanosec = stamp_nanosec;
  out.poses.reserve(path.points.size());
  for (const auto &p : path.points) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = out.header;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    out.poses.push_back(ps);
  }
  return out;
}


std::vector<GeometricPath> CgalHomotopyPlanner::generatePaths(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &goal,
    const std::vector<CircleObstacle> &obstacles,
    const Options &options) const {

  std::vector<GeometricPath> paths;


  if (segmentFreeOfCircles(start, goal, obstacles, options.safety_margin)) {
    GeometricPath gp; 
    gp.points = {start, goal}; 
    gp.length = (goal - start).norm(); 
    gp.homotopy_signature = "w=0";
    paths.push_back(gp);
    return paths;
  }

  if (obstacles.empty()) {
    GeometricPath gp; 
    gp.points = {start, goal}; 
    gp.length = (goal - start).norm(); 
    gp.homotopy_signature = "w=0";
    if (gp.length > 1e-6) paths.push_back(gp);
    return paths;
  }

  auto skeleton = buildObstacleAvoidingSkeleton(obstacles, options, start, goal);
  if (skeleton.vertices.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("CgalHomotopyPlanner"), "Failed to build skeleton");
    return paths;
  }

  Graph graph;
  graph.nodes = skeleton.vertices;
  graph.adj = skeleton.adjacency;
  
  std::unordered_map<long long,int> hmap; 
  hmap.reserve(graph.nodes.size() + 2);
  
  for (int i = 0; i < (int)graph.nodes.size(); ++i) {
    auto q = key2d(graph.nodes[i]);
    long long h = (q.first<<32) ^ (q.second & 0xffffffff);
    hmap[h] = i;
  }

  if (graph.nodes.size() < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("CgalHomotopyPlanner"), "Graph too small: %zu nodes", graph.nodes.size());
    return paths;
  }


  const int K_NEI = 5;
  std::vector<std::pair<double,int>> candS, candG;
  candS.reserve(graph.nodes.size()); 
  candG.reserve(graph.nodes.size());
  
  for (int i = 0; i < (int)graph.nodes.size(); ++i) {
    candS.push_back({(graph.nodes[i] - start).squaredNorm(), i});
    candG.push_back({(graph.nodes[i] - goal ).squaredNorm(), i});
  }
  
  std::partial_sort(candS.begin(), candS.begin()+std::min(K_NEI,(int)candS.size()), candS.end());
  std::partial_sort(candG.begin(), candG.begin()+std::min(K_NEI,(int)candG.size()), candG.end());
  
  int sId = getOrAddNode(start, graph, hmap);
  int gId = getOrAddNode(goal,  graph, hmap);
  graph.adj.resize(graph.nodes.size());
  
  int start_connections = connectStartGoalRobustly(start, sId, graph, obstacles, options.safety_margin);
  int goal_connections = connectStartGoalRobustly(goal, gId, graph, obstacles, options.safety_margin);
  
  if (start_connections == 0 || goal_connections == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("CgalHomotopyPlanner"), 
                 "Failed to connect start/goal to skeleton (start=%d, goal=%d connections)",
                 start_connections, goal_connections);
    return paths;
  }

  if (segmentFreeOfCircles(start, goal, obstacles, options.safety_margin)) {
    double w = (goal - start).norm();
    graph.adj[sId].push_back({gId, w});
  }


  std::vector<GeometricPath> collected;
  Graph work = graph;
  const int Kpaths = std::max<size_t>(1, options.max_number_of_paths);
  
  for (int k = 0; k < Kpaths; ++k) {
    auto idxPath = dijkstra(work, sId, gId);
    if (idxPath.empty()) break;
    
    std::vector<Eigen::Vector2d> poly; 
    poly.reserve(idxPath.size());
    for (int v : idxPath) poly.push_back(work.nodes[v]);
    
    if ((int)idxPath.size() >= 2) {
      int cutIdx = (int)idxPath.size()/2 - 1; 
      if (cutIdx < 0) cutIdx = 0;
      int u = idxPath[cutIdx], v = idxPath[cutIdx+1];
      auto &adjU = work.adj[u]; 
      adjU.erase(std::remove_if(adjU.begin(), adjU.end(), [&](auto &e){return e.first==v;}), adjU.end());
      auto &adjV = work.adj[v]; 
      adjV.erase(std::remove_if(adjV.begin(), adjV.end(), [&](auto &e){return e.first==u;}), adjV.end());
    }
    

    GeometricPath gp; 
    gp.points = std::move(poly); 
    gp.length = 0.0;
    for (size_t i = 1; i < gp.points.size(); ++i) {
      gp.length += (gp.points[i]-gp.points[i-1]).norm();
    }
    gp.homotopy_signature = computeHomotopySignature(gp.points, obstacles);
    collected.push_back(std::move(gp));
  }
  

  std::unordered_set<std::string> seen;
  for (auto &g : collected) {
    if (seen.insert(g.homotopy_signature).second) {
      if (options.enable_bezier_smoothing) {
        auto smoothed = smoothPathWithBezier(g.points, obstacles, options);
        if (validateSmoothedPath(smoothed, obstacles, options.safety_margin, options.smoothing_collision_step)) {
          g.points = std::move(smoothed);
        }

      }
      
      for (size_t i = 1; i < g.points.size(); ++i) {
        g.length += (g.points[i]-g.points[i-1]).norm();
      }
      paths.push_back(std::move(g));
    }
  }
  

  std::sort(paths.begin(), paths.end(), 
    [](const GeometricPath&a, const GeometricPath&b){return a.length < b.length;});
  if (paths.size() > options.max_number_of_paths) {
    paths.resize(options.max_number_of_paths);
  }
  return paths;
}


std::vector<CgalHomotopyPlanner::SkeletonEdge> CgalHomotopyPlanner::buildSkeletonEdges(
    const std::vector<CircleObstacle> &obstacles,
    const Options &options,
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &goal) const {
  std::vector<SkeletonEdge> out;
  auto skel = buildObstacleAvoidingSkeleton(obstacles, options, start, goal);
  out.reserve(skel.edges.size());
  for (const auto &e : skel.edges) {
    out.push_back({e.source, e.target});
  }
  return out;
}

std::vector<GeometricPath> CgalHomotopyPlanner::generatePaths(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &goal,
    const std::vector<CircleObstacle> &obstacles) const {
  Options o; return generatePaths(start, goal, obstacles, o);
}

CgalHomotopyPlanner::ApolloniusGraph CgalHomotopyPlanner::buildObstacleAvoidingSkeleton(
    const std::vector<CircleObstacle> &obstacles,
    const Options &options,
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &goal) const {
  
  ApolloniusGraph skeleton;
  
  if (obstacles.empty()) {
    return skeleton;
  }


  AG2 ag;
  for (const auto &c : obstacles) {
    double R = std::max(0.0, c.radius + options.safety_margin);
    ag.insert(Site(K::Point_2(c.center.x(), c.center.y()), R));
  }
  ag.is_valid();

  if (ag.number_of_vertices() == 0) {
    return skeleton;
  }


  VD vd(ag);
  

  std::unordered_map<long long, int> vertex_map;
  auto eigen2 = [](const K::Point_2 &p) { 
    return Eigen::Vector2d(CGAL::to_double(p.x()), CGAL::to_double(p.y())); 
  };

  double minx = std::numeric_limits<double>::max();
  double maxx = std::numeric_limits<double>::lowest(); 
  double miny = std::numeric_limits<double>::max();
  double maxy = std::numeric_limits<double>::lowest();
  
  for (const auto &o : obstacles) {
    double margin = o.radius + options.safety_margin + options.bbox_expansion_margin;
    minx = std::min(minx, o.center.x() - margin);
    maxx = std::max(maxx, o.center.x() + margin);
    miny = std::min(miny, o.center.y() - margin);
    maxy = std::max(maxy, o.center.y() + margin);
  }
  

  bool has_valid_start_goal = (start.norm() > 1e-9 || goal.norm() > 1e-9);
  if (has_valid_start_goal) {
    double extra_margin = options.bbox_expansion_margin;
    minx = std::min({minx, start.x() - extra_margin, goal.x() - extra_margin});
    maxx = std::max({maxx, start.x() + extra_margin, goal.x() + extra_margin});
    miny = std::min({miny, start.y() - extra_margin, goal.y() - extra_margin});
    maxy = std::max({maxy, start.y() + extra_margin, goal.y() + extra_margin});
  }
  

  auto in_bbox = [&](const Eigen::Vector2d &p) {
    return p.x() >= minx && p.x() <= maxx && p.y() >= miny && p.y() <= maxy;
  };
  

  auto is_reasonable = [](const Eigen::Vector2d &p) {
    const double MAX_COORD = 1e6;
    return std::abs(p.x()) < MAX_COORD && std::abs(p.y()) < MAX_COORD &&
           std::isfinite(p.x()) && std::isfinite(p.y());
  };


  for (auto eit = vd.halfedges_begin(); eit != vd.halfedges_end(); ++eit) {
  
    if (!eit->has_source() || !eit->has_target()) continue;
    
    const auto &ps = eit->source()->point();
    const auto &pt = eit->target()->point();
    
    Eigen::Vector2d source = eigen2(ps);
    Eigen::Vector2d target = eigen2(pt);
    
  
    if (!in_bbox(source) || !in_bbox(target) || 
        !is_reasonable(source) || !is_reasonable(target)) {
      continue;
    }
    
  
    double edge_length = (target - source).norm();
    const double MAX_EDGE_LENGTH = (maxx - minx + maxy - miny);
    if (edge_length < 1e-9 || edge_length > MAX_EDGE_LENGTH) {
      continue;
    }
    
  
    auto addVertex = [&](const Eigen::Vector2d &v) -> int {
      auto key = key2d(v);
      long long hash = (key.first << 32) ^ (key.second & 0xffffffff);
      auto it = vertex_map.find(hash);
      if (it != vertex_map.end()) {
        return it->second;
      }
      int id = skeleton.vertices.size();
      skeleton.vertices.push_back(v);
      vertex_map[hash] = id;
      return id;
    };

    (void)addVertex(source);
    (void)addVertex(target);
    

    double weight = (target - source).norm();
    
  
    ApolloniusEdge edge;
    edge.source = source;
    edge.target = target;
    edge.length = weight;
    skeleton.edges.push_back(edge);
  }


  skeleton.adjacency.resize(skeleton.vertices.size());
  for (size_t i = 0; i < skeleton.edges.size(); ++i) {
    const auto &edge = skeleton.edges[i];
    

    int source_idx = -1, target_idx = -1;
    for (size_t j = 0; j < skeleton.vertices.size(); ++j) {
      if ((skeleton.vertices[j] - edge.source).norm() < 1e-9) {
        source_idx = j;
      }
      if ((skeleton.vertices[j] - edge.target).norm() < 1e-9) {
        target_idx = j;
      }
    }
    
    if (source_idx >= 0 && target_idx >= 0) {
      skeleton.adjacency[source_idx].emplace_back(target_idx, edge.length);
      skeleton.adjacency[target_idx].emplace_back(source_idx, edge.length);
    }
  }

  return skeleton;
}


int CgalHomotopyPlanner::connectStartGoalRobustly(
    const Eigen::Vector2d &point,
    int point_id,
    Graph &graph,
    const std::vector<CircleObstacle> &obstacles,
    double safety_margin) const {
  
  int connections_made = 0;
  const int MAX_ATTEMPTS = std::min(10, (int)graph.nodes.size());
  

  std::vector<std::pair<double,int>> candidates;
  candidates.reserve(graph.nodes.size());
  
  for (int i = 0; i < (int)graph.nodes.size(); ++i) {
    if (i == point_id) continue;
    candidates.push_back({(graph.nodes[i] - point).squaredNorm(), i});
  }
  
  std::partial_sort(candidates.begin(), 
                   candidates.begin() + std::min(MAX_ATTEMPTS, (int)candidates.size()), 
                   candidates.end());
  

  for (int j = 0; j < std::min(MAX_ATTEMPTS, (int)candidates.size()); ++j) {
    int v = candidates[j].second;
    const Eigen::Vector2d &target = graph.nodes[v];
    
    if (segmentFreeOfCircles(point, target, obstacles, safety_margin)) {
      double w = (target - point).norm();
      graph.adj[point_id].push_back({v, w});
      graph.adj[v].push_back({point_id, w});
      connections_made++;
      
      if (connections_made >= 3) break;
    }
  }
  

  if (connections_made == 0) {
    for (const auto &obs : obstacles) {
      auto tangent_points = computeTangentPoints(point, obs, safety_margin * 1.5);
      
      for (const auto &tangent : tangent_points) {
      
        for (int j = 0; j < std::min(5, (int)candidates.size()); ++j) {
          int v = candidates[j].second;
          const Eigen::Vector2d &target = graph.nodes[v];
          

          if (segmentFreeOfCircles(point, tangent, obstacles, safety_margin) &&
              segmentFreeOfCircles(tangent, target, obstacles, safety_margin)) {
            
          
            int tangent_id = graph.nodes.size();
            graph.nodes.push_back(tangent);
            graph.adj.emplace_back();
            

            double w1 = (tangent - point).norm();
            double w2 = (target - tangent).norm();
            
            graph.adj[point_id].push_back({tangent_id, w1});
            graph.adj[tangent_id].push_back({point_id, w1});
            graph.adj[tangent_id].push_back({v, w2});
            graph.adj[v].push_back({tangent_id, w2});
            
            connections_made++;
            if (connections_made >= 2) break;
          }
        }
        if (connections_made >= 2) break;
      }
      if (connections_made >= 2) break;
    }
  }
  

  if (connections_made == 0) {
    std::vector<std::pair<double,int>> extended_candidates;
    extended_candidates.reserve(graph.nodes.size());
    
  
    for (int i = 0; i < (int)graph.nodes.size(); ++i) {
      if (i == point_id) continue;
      double dist = (graph.nodes[i] - point).norm();
      if (dist <= 50.0) {
        extended_candidates.push_back({dist * dist, i});
      }
    }
    
    std::sort(extended_candidates.begin(), extended_candidates.end());
    
    for (const auto &candidate : extended_candidates) {
      int v = candidate.second;
      const Eigen::Vector2d &target = graph.nodes[v];
      
      if (segmentFreeOfCircles(point, target, obstacles, safety_margin * 0.8)) {
        double w = (target - point).norm();
        graph.adj[point_id].push_back({v, w});
        graph.adj[v].push_back({point_id, w});
        connections_made++;
        
        if (connections_made >= 1) break;
      }
    }
  }
  
  return connections_made;
}

std::vector<Eigen::Vector2d> CgalHomotopyPlanner::computeTangentPoints(
    const Eigen::Vector2d &point,
    const CircleObstacle &obstacle,
    double safety_margin) const {
  
  std::vector<Eigen::Vector2d> tangents;
  
  Eigen::Vector2d center = obstacle.center;
  double radius = obstacle.radius + safety_margin;
  
  Eigen::Vector2d pc = center - point;
  double d = pc.norm();
  

  if (d <= radius + 1e-6) {
    return tangents;
  }
  

  double theta = std::asin(radius / d);
  double alpha = std::atan2(pc.y(), pc.x());
  

  double angle1 = alpha + theta;
  double angle2 = alpha - theta;
  

  for (double angle : {angle1, angle2}) {
    Eigen::Vector2d direction(std::cos(angle), std::sin(angle));
    Eigen::Vector2d tangent_point = center + radius * direction;
    tangents.push_back(tangent_point);
  }
  
  return tangents;
}

std::vector<Eigen::Vector2d> CgalHomotopyPlanner::smoothPathWithBezier(
    const std::vector<Eigen::Vector2d> &path,
    const std::vector<CircleObstacle> & /*obstacles*/,
    const Options &options) const {
  
  if (path.size() <= 2 || options.bezier_samples_per_segment <= 1) {
    return path;
  }
  
  std::vector<Eigen::Vector2d> smoothed;
  
  auto sampleCubicBezier = [](const Eigen::Vector2d &P0,
                             const Eigen::Vector2d &P1,
                             const Eigen::Vector2d &P2,
                             const Eigen::Vector2d &P3,
                             int n) {
    std::vector<Eigen::Vector2d> samples;
    for (int i = 1; i <= n; ++i) {
      double t = (double)i / (double)n;
      double u = 1.0 - t;
      Eigen::Vector2d point = u*u*u*P0 + 3*u*u*t*P1 + 3*u*t*t*P2 + t*t*t*P3;
      samples.push_back(point);
    }
    return samples;
  };
  
  auto appendUniquePoint = [&](const Eigen::Vector2d &point) {
    if (smoothed.empty() || (smoothed.back() - point).norm() > 1e-9) {
      smoothed.push_back(point);
    }
  };
  
  const double tension = 0.5;
  appendUniquePoint(path.front());
  
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    Eigen::Vector2d P0 = (i == 0) ? path[i] : path[i - 1];
    Eigen::Vector2d P1 = path[i];
    Eigen::Vector2d P2 = path[i + 1];
    Eigen::Vector2d P3 = (i + 2 < path.size()) ? path[i + 2] : path[i + 1];
    
  
    Eigen::Vector2d B0 = P1;
    Eigen::Vector2d B3 = P2;
    Eigen::Vector2d B1 = P1 + (P2 - P0) * (tension / 3.0);
    Eigen::Vector2d B2 = P2 - (P3 - P1) * (tension / 3.0);
    
    auto samples = sampleCubicBezier(B0, B1, B2, B3, options.bezier_samples_per_segment);
    for (const auto &sample : samples) {
      appendUniquePoint(sample);
    }
  }
  
  return smoothed;
}

bool CgalHomotopyPlanner::validateSmoothedPath(
    const std::vector<Eigen::Vector2d> &path,
    const std::vector<CircleObstacle> &obstacles,
    double safety_margin,
    double /*step_size*/) const {
  
  if (path.size() < 2) return true;
  
  for (size_t i = 1; i < path.size(); ++i) {
    if (!segmentFreeOfCircles(path[i-1], path[i], obstacles, safety_margin)) {
      return false;
    }
  }
  
  return true;
}

}




