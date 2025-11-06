#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <random>
#include <vector>

namespace ghost_planner
{

class RandomCylinderPublisher : public rclcpp::Node
{
public:
    RandomCylinderPublisher(const rclcpp::NodeOptions & options);

private:
    void publish_obstacles();

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double map_x_size_;
    double map_y_size_;
    double map_height_;
    std::string frame_id_;

    int num_obstacles_;
    double min_radius_;
    double max_radius_;

    bool use_fixed_map_;
    std::vector<double> fixed_x_;
    std::vector<double> fixed_y_;
    std::vector<double> fixed_r_;

    int random_seed_;
    std::mt19937 rng_;
};

} // namespace ghost_planner
