#include "ghost planner/random_cylinder_publisher.hpp"

namespace ghost_planner
{

RandomCylinderPublisher::RandomCylinderPublisher(const rclcpp::NodeOptions & options)
    : Node("random_cylinder_publisher", options),
      rng_(std::random_device{}())
{
    RCLCPP_INFO(this->get_logger(), "Initializing Random Cylinder Publisher...");

    map_x_size_ = this->declare_parameter<double>("map_size.x", 20.0);
    map_y_size_ = this->declare_parameter<double>("map_size.y", 20.0);
    map_height_ = this->declare_parameter<double>("map_size.z", 1.0);
    num_obstacles_ = this->declare_parameter<int>("obstacles.number", 15);
    min_radius_ = this->declare_parameter<double>("obstacles.radius_min", 0.5);
    max_radius_ = this->declare_parameter<double>("obstacles.radius_max", 1.5);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    use_fixed_map_ = this->declare_parameter<bool>("obstacles.use_fixed", false);
    fixed_x_ = this->declare_parameter<std::vector<double>>("obstacles.fixed_x", std::vector<double>());
    fixed_y_ = this->declare_parameter<std::vector<double>>("obstacles.fixed_y", std::vector<double>());
    fixed_r_ = this->declare_parameter<std::vector<double>>("obstacles.fixed_r", std::vector<double>());
    random_seed_ = this->declare_parameter<int>("random_seed", -1);

    if (random_seed_ >= 0) {
        rng_.seed(static_cast<uint32_t>(random_seed_));
    }
    
    obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles", rclcpp::QoS(1).transient_local());

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() -> void {
            this->publish_obstacles();
            this->timer_->cancel();
        });
}

void RandomCylinderPublisher::publish_obstacles()
{
    visualization_msgs::msg::MarkerArray marker_array;

    struct Disk { double x, y, r; };
    std::vector<Disk> placed;
    placed.reserve(num_obstacles_);

    if (use_fixed_map_) {
        if (fixed_x_.size() != fixed_y_.size() || fixed_x_.size() != fixed_r_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Fixed obstacle lists sizes mismatch: x=%zu, y=%zu, r=%zu", fixed_x_.size(), fixed_y_.size(), fixed_r_.size());
        } else {
            marker_array.markers.reserve(fixed_x_.size());
            for (size_t i = 0; i < fixed_x_.size(); ++i) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = frame_id_;
                marker.header.stamp    = this->get_clock()->now();
                marker.ns   = "obstacles";
                marker.id   = static_cast<int>(i);
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.pose.position.x = fixed_x_[i];
                marker.pose.position.y = fixed_y_[i];
                marker.pose.position.z = map_height_ * 0.5;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                double d = fixed_r_[i] * 2.0;
                marker.scale.x = d;
                marker.scale.y = d;
                marker.scale.z = map_height_;

                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;
                marker.color.a = 1.0f;

                marker.lifetime = rclcpp::Duration(0, 0);
                marker_array.markers.push_back(std::move(marker));
            }
            obstacles_pub_->publish(marker_array);
            RCLCPP_INFO(this->get_logger(), "Published fixed obstacle map: %zu obstacles.", marker_array.markers.size());
            return;
        }
    }

    std::uniform_real_distribution<double> r_dist(min_radius_, max_radius_);
    constexpr double kCornerClearance = 3.0;
    constexpr int kMaxTriesPerObstacle = 2000;
    constexpr double kShrinkFactor = 0.95;
    constexpr double kEps = 1e-6;

    while (static_cast<int>(placed.size()) < num_obstacles_) {
        double r_try = r_dist(rng_);
        bool placed_ok = false;

        for (int attempt = 0; attempt < kMaxTriesPerObstacle && !placed_ok; ++attempt) {
            std::uniform_real_distribution<double> x_dist(r_try, std::max(r_try, map_x_size_ - r_try));
            std::uniform_real_distribution<double> y_dist(r_try, std::max(r_try, map_y_size_ - r_try));

            double x = x_dist(rng_);
            double y = y_dist(rng_);

            bool in_lower_left  = (x - r_try < kCornerClearance && y - r_try < kCornerClearance);
            bool in_upper_right = (x + r_try > map_x_size_ - kCornerClearance && y + r_try > map_y_size_ - kCornerClearance);
            if (in_lower_left || in_upper_right) {
                goto next_attempt;
            }

            {
                bool collide = false;
                for (const auto& d : placed) {
                    double dx = x - d.x;
                    double dy = y - d.y;
                    double min_dist = d.r + r_try - kEps;
                    if (dx*dx + dy*dy < min_dist * min_dist) {
                        collide = true;
                        break;
                    }
                }
                if (collide) goto next_attempt;
            }

            placed.push_back({x, y, r_try});
            placed_ok = true;
            break;

        next_attempt:
            if ((attempt + 1) % 200 == 0 && r_try > min_radius_) {
                r_try = std::max(min_radius_, r_try * kShrinkFactor);
            }
        }

        if (!placed_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not place all obstacles without overlap at current settings. "
                        "Placed %zu/%d so far. Trying smaller radii near min_radius.",
                        placed.size(), num_obstacles_);

            double r_force = min_radius_;
            for (int attempt = 0; attempt < kMaxTriesPerObstacle && !placed_ok; ++attempt) {
                std::uniform_real_distribution<double> x_dist(r_force, std::max(r_force, map_x_size_ - r_force));
                std::uniform_real_distribution<double> y_dist(r_force, std::max(r_force, map_y_size_ - r_force));
                double x = x_dist(rng_);
                double y = y_dist(rng_);
                bool in_lower_left  = (x - r_force < kCornerClearance && y - r_force < kCornerClearance);
                bool in_upper_right = (x + r_force > map_x_size_ - kCornerClearance && y + r_force > map_y_size_ - kCornerClearance);
                if (in_lower_left || in_upper_right) continue;

                bool collide = false;
                for (const auto& d : placed) {
                    double dx = x - d.x;
                    double dy = y - d.y;
                    double min_dist = d.r + r_force - kEps;
                    if (dx*dx + dy*dy < min_dist * min_dist) { collide = true; break; }
                }
                if (!collide) { placed.push_back({x, y, r_force}); placed_ok = true; break; }
            }

            if (!placed_ok) {
                RCLCPP_ERROR(this->get_logger(),
                             "Configuration too dense: cannot place %d non-overlapping cylinders "
                             "within map %.2fx%.2f with min_radius=%.2f. Placed %zu.",
                             num_obstacles_, map_x_size_, map_y_size_, min_radius_, placed.size());
                break;
            }
        }
    }

    marker_array.markers.reserve(placed.size());
    for (int i = 0; i < static_cast<int>(placed.size()); ++i) {
        const auto& d = placed[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp    = this->get_clock()->now();
        marker.ns   = "obstacles";
        marker.id   = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = d.x;
        marker.pose.position.y = d.y;
        marker.pose.position.z = map_height_ * 0.5;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = d.r * 2.0;
        marker.scale.y = d.r * 2.0;
        marker.scale.z = map_height_;

        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0, 0);

        marker_array.markers.push_back(std::move(marker));
    }

    obstacles_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published %zu obstacles (requested %d).",
                marker_array.markers.size(), num_obstacles_);
}

} // namespace ghost_planner

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<ghost_planner::RandomCylinderPublisher>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
