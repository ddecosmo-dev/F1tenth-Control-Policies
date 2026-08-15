// RRT assignment

#pragma once

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <array>
#include <cstdint>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to this struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost = 0.0; // only used for RRT*
    int parent = -1; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;


class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // Grid parameters (from visualization code)
    static constexpr double pi_ = 3.14159265358979323846;
    static constexpr double GRID_RESOLUTION  = 0.1;  // meters per cell
    static constexpr double GRID_SIZE        = 10.0; // width/height of grid, meters
    static constexpr double INFLATION_RADIUS = 0.3; // inflate occupied cells by (car) radius, meters
    static constexpr int GRID_CELLS          = (int)(GRID_SIZE / GRID_RESOLUTION);

    // Occupancy grid and state
    std::array<uint8_t, GRID_CELLS * GRID_CELLS> occupancy_grid_{};
    std::array<uint8_t, GRID_CELLS * GRID_CELLS> freespace_mask_{};
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double car_x_ = 0.0;
    double car_y_ = 0.0;
    double car_yaw_ = 0.0;
    bool odom_ready_ = false;
    bool scan_ready_ = false;

    // Planner parameters/state
    double goal_threshold_ = 0.5;
    double step_size_ = 0.5;
    int max_iterations_ = 500;
    double max_expansion_dist_ = 1.0;
    double neighborhood_radius_ = 0.5;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    bool goal_ready_ = false;
    int tree_publish_stride_ = 3;
    int plan_cycle_count_ = 0;

    // publishers and subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rrt_tree_publisher_;

    // random generator
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;    

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    // updates local planning goal from pure pursuit target topic
    void goal_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal_msg);
    // publish occupancy grid for visualization
    void publish_occupancy_grid(const rclcpp::Time &stamp, const std::array<uint8_t, GRID_CELLS * GRID_CELLS> &raw_occupied);
    
    // collision helper - checks if a world coord is free 
    bool is_free(double x, double y) const {
        // Check if a world-coordinate point lands in a free occupancy cell.
        int col = static_cast<int>((x - origin_x_) / GRID_RESOLUTION);
        int row = static_cast<int>((y - origin_y_) / GRID_RESOLUTION);

        if (col < 0 || col >= GRID_CELLS || row < 0 || row >= GRID_CELLS) {
            return false;
        }
        return occupancy_grid_[row * GRID_CELLS + col] == 0 && freespace_mask_[row * GRID_CELLS + col] == 1;
    }

    // collision helper - checks if a grid cell is free
    bool is_cell_free(int row, int col) const {
        // Check if a grid index is in-bounds and unoccupied.
        if (col < 0 || col >= GRID_CELLS || row < 0 || row >= GRID_CELLS) {
            return false;
        }
        return occupancy_grid_[row * GRID_CELLS + col] == 0 && freespace_mask_[row * GRID_CELLS + col] == 1;
    }

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

};
