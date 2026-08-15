#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <array>
#include <fstream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

class PurePursuit : public rclcpp::Node
{

private:
static constexpr double HF_pi = 3.14159265358979323846;

std::vector<std::array<double, 2>> csv_waypoints_; // Static CSV target waypoints (never change)
std::vector<std::array<double, 2>> rrt_path_; // Current RRT path for steering (updates when new path received)
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber;
rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr rrt_path_subscriber;
rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;

double lookahead_distance_;
double max_steering_angle_;
double speed = 0.0;
int current_waypoint_index_ = 0;
int lookahead_index_offset_ = 3;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        waypoints_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/pure_pursuit/waypoints", rclcpp::QoS(1).transient_local());
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/pure_pursuit/target_point", rclcpp::QoS(1));
        rrt_path_subscriber = this->create_subscription<nav_msgs::msg::Path>(
            "/rrt/path", 10, std::bind(&PurePursuit::rrt_path_callback, this, std::placeholders::_1));
        this->declare_parameter<double>("lookahead_distance", 3.0);
        this->declare_parameter<double>("max_steering_angle", 60.0);
        this->declare_parameter<double>("speed", 4.0);
        this->declare_parameter<int>("lookahead_index_offset", 3);
        this->declare_parameter<std::string>(
            "waypoint_csv", "/home/f1humble/sim_ws/src/lab06_pkg/waypoints.csv");

        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double() * HF_pi / 180.0; // convert to radians
        speed = this->get_parameter("speed").as_double();
        lookahead_index_offset_ = std::max<int64_t>(0, this->get_parameter("lookahead_index_offset").as_int());
        const auto waypoint_csv = this->get_parameter("waypoint_csv").as_string();


        if (!load_waypoints_from_csv(waypoint_csv) || csv_waypoints_.size() < 2) {
            RCLCPP_FATAL(this->get_logger(),
                "Failed to load at least 2 waypoints from CSV: %s", waypoint_csv.c_str());
            throw std::runtime_error("Invalid waypoint CSV");
        }

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node started.");
        RCLCPP_INFO(this->get_logger(), "Loaded %zu target waypoints from: %s", csv_waypoints_.size(), waypoint_csv.c_str());
        publish_all_waypoints();
    }

    bool load_waypoints_from_csv(const std::string &csv_path)
    {
        std::ifstream infile(csv_path);
        if (!infile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open waypoint CSV: %s", csv_path.c_str());
            return false;
        }

        std::vector<std::array<double, 2>> loaded;
        std::string line;
        while (std::getline(infile, line)) {
            if (line.empty()) {
                continue;
            }

            std::stringstream ss(line);
            std::string x_str;
            std::string y_str;

            if (!std::getline(ss, x_str, ',')) {
                continue;
            }
            if (!std::getline(ss, y_str)) {
                continue;
            }

            try {
                const double x = std::stod(x_str);
                const double y = std::stod(y_str);
                loaded.push_back({x, y});
            } catch (const std::exception &) {
                // Skip headers or malformed rows.
                continue;
            }
        }

        if (loaded.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid waypoints found in CSV: %s", csv_path.c_str());
            return false;
        }

        csv_waypoints_ = loaded;
        return true;
    }

    void publish_all_waypoints()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";
        for (const auto & wp : csv_waypoints_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path_msg.header;
            ps.pose.position.x = wp[0];
            ps.pose.position.y = wp[1];
            ps.pose.position.z = 0.0;
            ps.pose.orientation.w = 1.0;
            path_msg.poses.push_back(ps);
        }
        waypoints_pub_->publish(path_msg);
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        const auto &q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        // Check if we've reached the current target CSV waypoint
        double dx_curr = csv_waypoints_[current_waypoint_index_][0] - x;
        double dy_curr = csv_waypoints_[current_waypoint_index_][1] - y;
        double dist_to_current = std::sqrt(dx_curr * dx_curr + dy_curr * dy_curr);
        
        // if close enough, advance to next CSV waypoint
        //TODO: change to param
        if (dist_to_current < 1.0) {
            current_waypoint_index_ = (current_waypoint_index_ + 1) % csv_waypoints_.size();
        }
        
        int target = current_waypoint_index_;
        
        // Find lookahead target several waypoints ahead in CSV (default: 2)
        int lookahead_idx = (target + lookahead_index_offset_) % csv_waypoints_.size();
        
        // Publish lookahead target to RRT as the goal
        geometry_msgs::msg::PointStamped target_msg;
        target_msg.header.stamp = this->get_clock()->now();
        target_msg.header.frame_id = "map";
        target_msg.point.x = csv_waypoints_[lookahead_idx][0];
        target_msg.point.y = csv_waypoints_[lookahead_idx][1];
        target_msg.point.z = 0.0;
        target_pub_->publish(target_msg);

        // // Steer using RRT path
        // double goal_x = csv_waypoints_[lookahead_idx][0];
        // double goal_y = csv_waypoints_[lookahead_idx][1];
        
        // if (!rrt_path_.empty()) {
        //     // Follow RRT path - find first point ahead of car distance
        //     bool found_ahead = false;
        //     for (const auto &node : rrt_path_) {
        //         double dx = node[0] - x;
        //         double dy = node[1] - y;
        //         double dist = std::sqrt(dx * dx + dy * dy);
        //         if (dist > 0.1) { // Small threshold to skip current position
        //             goal_x = node[0];
        //             goal_y = node[1];
        //             found_ahead = true;
        //             break;
        //         }
        //     }
        //     if (!found_ahead && !rrt_path_.empty()) {
        //         goal_x = rrt_path_.back()[0];
        //         goal_y = rrt_path_.back()[1];
        //     }
        // }

        // RRT* path following - if path exists, follow it
        if (rrt_path_.empty()) {
            // No RRT* path: hold path, wait for next callback
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = this->get_clock()->now();
            drive_msg.header.frame_id = "base_link";
            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;
            drive_publisher->publish(drive_msg);
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "No RRT* path available, commanding stop");
            return;
        }

        // find first point ahead of the car
        double goal_x = rrt_path_.back()[0];
        double goal_y = rrt_path_.back()[1];
        for (const auto &node : rrt_path_) {
            double dx = node[0] - x;
            double dy = node[1] - y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist > 0.1) {
                goal_x = node[0];
                goal_y = node[1];
                break;
            }
        }

        // transform to vehicle frame
        double dx = goal_x - x;
        double dy = goal_y - y;

        // transform goal to local frame
        double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);

        // Calculate curvature/steering angle
        double steering_angle = (2.0 * local_y) / (lookahead_distance_ * lookahead_distance_);
        steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));

        // Publish drive message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;
        drive_publisher->publish(drive_msg);
    }

    void rrt_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Received empty RRT* path, keeping previous path");
            return;
        }

        // update RRT path for steering
        rrt_path_.clear();
        for (const auto &pose : msg->poses) {
            rrt_path_.push_back({pose.pose.position.x, pose.pose.position.y});
        }

        RCLCPP_INFO(this->get_logger(), "Updated RRT path with %zu points", rrt_path_.size());
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
} 