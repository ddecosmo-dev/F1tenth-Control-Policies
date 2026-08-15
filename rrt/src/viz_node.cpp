// original code by Henry A. Feldhaus
// Lab 06: Motion Planning; F1TENTH 16-663 CMU, Spring 2026
// VizNode

#include <array>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// ---------------------------------------------------------------------------
// VizNode
//
// Subscribes to:
//   /ego_racecar/odom          - vehicle pose (nav_msgs/Odometry)
//   /pure_pursuit/target_point - lookahead target published by pure_pursuit_node
//                                (geometry_msgs/PointStamped)
//   /rrt/path                  - planned path from RRT (nav_msgs/Path)
//
// Publishes:
//   /viz/markers               - combined MarkerArray for RViz
// ---------------------------------------------------------------------------

class VizNode : public rclcpp::Node
{
public:
    VizNode() : Node("viz_node")
    {
        // subs
        target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/pure_pursuit/target_point", 10,
            std::bind(&VizNode::target_callback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/rrt/path", 10,
            std::bind(&VizNode::path_callback, this, std::placeholders::_1));

        waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/pure_pursuit/waypoints", rclcpp::QoS(1).transient_local(),
            std::bind(&VizNode::waypoints_callback, this, std::placeholders::_1));

        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/rrt/occupancy_grid", 10,
            std::bind(&VizNode::occupancy_grid_callback, this, std::placeholders::_1));

        // pubs - separate publishers for each visualization type
        waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/waypoints", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/viz/path", 10);
        grid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/viz/occupancy_grid", 1);

        RCLCPP_INFO(this->get_logger(), "VizNode started.");
    }

private:
    void waypoints_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        waypoints_.clear();
        for (const auto & pose : msg->poses) {
            waypoints_.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints.", waypoints_.size());

        // Pre-build a single SPHERE_LIST for all waypoints (background, black).
        // This is computed once so target_callback only needs to publish 3 markers total.
        waypoints_bg_marker_ = visualization_msgs::msg::Marker();
        waypoints_bg_marker_.header.frame_id = "map";
        waypoints_bg_marker_.ns = "waypoints_bg";
        waypoints_bg_marker_.id = 0;
        waypoints_bg_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        waypoints_bg_marker_.action = visualization_msgs::msg::Marker::ADD;
        waypoints_bg_marker_.pose.orientation.w = 1.0;
        waypoints_bg_marker_.scale.x = 0.1;
        waypoints_bg_marker_.scale.y = 0.1;
        waypoints_bg_marker_.scale.z = 0.1;
        waypoints_bg_marker_.color.r = 0.0;
        waypoints_bg_marker_.color.g = 0.0;
        waypoints_bg_marker_.color.b = 0.0;
        waypoints_bg_marker_.color.a = 1.0;
        for (const auto & wp : waypoints_) {
            geometry_msgs::msg::Point p;
            p.x = wp[0]; p.y = wp[1]; p.z = 0.0;
            waypoints_bg_marker_.points.push_back(p);
        }
    }

    void target_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
    {
        if (waypoints_.empty()) { 
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waypoints not yet loaded");
            return; 
        }

        auto now = this->get_clock()->now();

        // Background list (all waypoints, black) — pre-built, just stamp it
        waypoints_bg_marker_.header.stamp = now;

        // Current goal waypoint: green sphere
        visualization_msgs::msg::Marker current_marker;
        current_marker.header.frame_id = "map";
        current_marker.header.stamp = now;
        current_marker.ns = "waypoints_current";
        current_marker.id = 0;
        current_marker.type = visualization_msgs::msg::Marker::SPHERE;
        current_marker.action = visualization_msgs::msg::Marker::ADD;
        current_marker.pose.position.x = msg->point.x;
        current_marker.pose.position.y = msg->point.y;
        current_marker.pose.position.z = 0.0;
        current_marker.pose.orientation.w = 1.0;
        current_marker.scale.x = 0.2;
        current_marker.scale.y = 0.2;
        current_marker.scale.z = 0.2;
        current_marker.color.r = 0.0;
        current_marker.color.g = 1.0;
        current_marker.color.b = 0.0;
        current_marker.color.a = 1.0;

        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(waypoints_bg_marker_);
        marker_array.markers.push_back(current_marker);
        // marker_array.markers.push_back(next_marker);
        waypoint_marker_pub_->publish(marker_array);
    }

    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        if (msg->poses.empty()) {
            visualization_msgs::msg::Marker clear_marker;
            clear_marker.header.frame_id = "map";
            clear_marker.header.stamp = this->get_clock()->now();
            clear_marker.ns = "rrt_planned_path";
            clear_marker.id = 0;
            clear_marker.action = visualization_msgs::msg::Marker::DELETE;
            path_pub_->publish(clear_marker);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Received empty RRT path.");
            return;
        }

        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->get_clock()->now();
        path_marker.ns = "rrt_planned_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Bright green line for the final path
        path_marker.scale.x = 0.15;  // Thicker line
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;   // Bright green
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;   // Full opacity

        for (const auto & pose_stamped : msg->poses) {
            path_marker.points.push_back(pose_stamped.pose.position);
        }

        path_pub_->publish(path_marker);
    }

    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker occupied_marker;
        visualization_msgs::msg::Marker inflated_marker;

        occupied_marker.header.frame_id = msg->header.frame_id;
        occupied_marker.header.stamp = this->get_clock()->now();
        occupied_marker.ns = "rrt_occupied";
        occupied_marker.id = 0;
        occupied_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_marker.action = visualization_msgs::msg::Marker::ADD;
        occupied_marker.pose.orientation.w = 1.0;
        occupied_marker.scale.x = msg->info.resolution;
        occupied_marker.scale.y = msg->info.resolution;
        occupied_marker.scale.z = 0.02;
        occupied_marker.color.r = 0.0;
        occupied_marker.color.g = 0.0;
        occupied_marker.color.b = 0.0;
        occupied_marker.color.a = 0.9;

        inflated_marker = occupied_marker;
        inflated_marker.ns = "rrt_inflated";
        inflated_marker.id = 0;
        inflated_marker.color.r = 1.0;
        inflated_marker.color.g = 0.0;
        inflated_marker.color.b = 0.0;
        inflated_marker.color.a = 0.8;

        const int width = static_cast<int>(msg->info.width);
        const int height = static_cast<int>(msg->info.height);
        const double res = msg->info.resolution;
        const double ox = msg->info.origin.position.x;
        const double oy = msg->info.origin.position.y;

        for (int row = 0; row < height; ++row)
        {
            for (int col = 0; col < width; ++col)
            {
                const int idx = row * width + col;
                const int value = msg->data[idx];
                if (value <= 0)
                {
                    continue;
                }

                geometry_msgs::msg::Point p;
                p.x = ox + (col + 0.5) * res;
                p.y = oy + (row + 0.5) * res;
                p.z = 0.0;

                if (value >= 90)
                {
                    occupied_marker.points.push_back(p);
                }
                else
                {
                    inflated_marker.points.push_back(p);
                }
            }
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "Received occupancy grid; occupancy markers are active.");
        marker_array.markers.push_back(inflated_marker);
        marker_array.markers.push_back(occupied_marker);
        grid_pub_->publish(marker_array);
    }

    // pub/sub members
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr      target_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                   path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                   waypoints_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr          occupancy_grid_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr     waypoint_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr           path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr      grid_pub_;

    std::vector<std::array<double, 2>> waypoints_;
    visualization_msgs::msg::Marker waypoints_bg_marker_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VizNode>());
    rclcpp::shutdown();
    return 0;
}