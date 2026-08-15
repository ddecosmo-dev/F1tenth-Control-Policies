// original code by Henry A. Feldhaus
// edited from starter skeleton
// Lab 06: Motion Planning; F1TENTH 16-663 CMU, Spring 2026
// RRT

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    string goal_topic = "/pure_pursuit/target_point";

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic, 1, std::bind(&RRT::goal_callback, this, std::placeholders::_1));

    // create a occupancy grid
    auto grid_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/rrt/occupancy_grid", grid_qos);

    // path publisher - regular QoS for real-time updates
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/rrt/path", 10);

    // tree publisher - throttled to prevent visualization lag
    rrt_tree_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rrt/tree", 1);

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //    publish occupancy grid message

    if (!odom_ready_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odom before publishing grid.");
        return;
    }

    origin_x_ = car_x_ - (GRID_SIZE / 2.0); // set car at center of occ grid
    origin_y_ = car_y_ - (GRID_SIZE / 2.0);

    occupancy_grid_.fill(0); // 1 = occupied/inflated, 0 = free of obstacle
    freespace_mask_.fill(0); // 1 = observed free by LiDAR ray tracing
    int cols = GRID_CELLS;
    int car_col = static_cast<int>((car_x_ - origin_x_) / GRID_RESOLUTION);
    int car_row = static_cast<int>((car_y_ - origin_y_) / GRID_RESOLUTION);

    // process scan and update occupancy grid
    constexpr size_t BEAM_STRIDE = 2; // only sample every other beam for efficiency on jetson
    for (size_t i = 0; i < scan_msg->ranges.size(); i += BEAM_STRIDE)
    {
        const double raw_range = scan_msg->ranges[i];
        if (std::isnan(raw_range))
        {
            continue; // skip invalid measurements
        }

        const double beam_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        if (std::abs(beam_angle) > (pi_ / 2.0))
        {
            continue; // only keep forward-facing 180 degree FOV
        }

        const bool hit_obstacle = std::isfinite(raw_range) &&
                                  raw_range >= scan_msg->range_min &&
                                  raw_range <= scan_msg->range_max;
        const double range = hit_obstacle ? raw_range : scan_msg->range_max;

        double world_x = car_x_ + range * std::cos(car_yaw_ + beam_angle);
        double world_y = car_y_ + range * std::sin(car_yaw_ + beam_angle);

        int col = static_cast<int>((world_x - origin_x_) / GRID_RESOLUTION);
        int row = static_cast<int>((world_y - origin_y_) / GRID_RESOLUTION);

        // Mark sensed free cells along each beam until the endpoint.
        int x0 = car_col;
        int y0 = car_row;
        int x1 = col;
        int y1 = row;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            bool at_end = (x0 == x1 && y0 == y1);
            if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < cols) {
                if (!(hit_obstacle && at_end)) {
                    freespace_mask_[y0 * cols + x0] = 1;
                }
            }
            if (at_end) {
                break;
            }

            int err2 = 2 * err;
            if (err2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (err2 < dx) {
                err += dx;
                y0 += sy;
            }
        }

        if (hit_obstacle && col >= 0 && col < cols && row >= 0 && row < cols)
        {
            occupancy_grid_[row * cols + col] = 1; // mark cell as occupied
            freespace_mask_[row * cols + col] = 0;
        }
    }

    scan_ready_ = true;

    // inflation - snapshot pre-inflation grid to avoid chained inflation
    constexpr int R = (int)(INFLATION_RADIUS / GRID_RESOLUTION);
    auto snapshot = occupancy_grid_;
    for (int i = 0; i < GRID_CELLS * GRID_CELLS; ++i)
    {
        if (snapshot[i] == 0) { continue; }
        int cr = i / GRID_CELLS; // row
        int cc = i % GRID_CELLS; // col

        for (int dr = -R; dr <= R; ++dr)
        // use R as check for original occupation, update actual grid
        {
            for (int dc = -R; dc <= R; ++dc)
            {
                if (dr * dr + dc * dc > R * R) { continue; } // circular inflation, check/omit corners
                int r = cr + dr;
                int c = cc + dc;
                if (r >= 0 && r < GRID_CELLS && c >= 0 && c < GRID_CELLS)
                {
                    occupancy_grid_[r * GRID_CELLS + c] = 1;
                }
            }
        }
    }

    publish_occupancy_grid(scan_msg->header.stamp, snapshot);
}

void RRT::goal_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal_msg) {
    // Update local planning goal from pure pursuit target point.
    goal_x_ = goal_msg->point.x;
    goal_y_ = goal_msg->point.y;
    goal_ready_ = true;
}

void RRT::publish_occupancy_grid(const rclcpp::Time &stamp, const std::array<uint8_t, GRID_CELLS * GRID_CELLS> &raw_occupied)
{
    // Publishes the occupancy grid for visualization
    // -1 unknown (not rendered), 50 inflated-only, 100 physically occupied (walls)

    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = stamp;
    grid_msg.header.frame_id = "map";

    // format occupancy grid message
    grid_msg.info.map_load_time = stamp;
    grid_msg.info.resolution = GRID_RESOLUTION;
    grid_msg.info.width = GRID_CELLS;
    grid_msg.info.height = GRID_CELLS;
    grid_msg.info.origin.position.x = origin_x_;
    grid_msg.info.origin.position.y = origin_y_;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.x = 0.0;
    grid_msg.info.origin.orientation.y = 0.0;
    grid_msg.info.origin.orientation.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    // populate data based on current grid state
    grid_msg.data.assign(occupancy_grid_.size(), -1);
    for (size_t i = 0; i < occupancy_grid_.size(); ++i) {
        if (raw_occupied[i]) {
            grid_msg.data[i] = 100;
        } else if (occupancy_grid_[i]) {
            grid_msg.data[i] = 50;
        }
    }
    grid_publisher_->publish(grid_msg);
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    car_x_ = pose_msg->pose.pose.position.x;
    car_y_ = pose_msg->pose.pose.position.y;
    
    const auto &q = pose_msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll_, pitch_;
    tf2::Matrix3x3(quat).getRPY(roll_, pitch_, car_yaw_);
    // car_yaw_ = std::atan2(
    //     2.0 * (q.w * q.z + q.x * q.y),
    //     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_ready_ = true;

    if (!goal_ready_) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /pure_pursuit/target_point goal."); return;}
    if (!scan_ready_) {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for scan before planning."); return;}

    tree.push_back({car_x_, car_y_, 0.0, -1, true}); // add root node (car position)
    tree.reserve(static_cast<size_t>(max_iterations_) + 1); // more otimizing for jetson

    bool path_found = false;
    std::vector<RRT_Node> found_path;
    int best_goal_idx = 0;
    double best_goal_dist_sq = (car_x_ - goal_x_) * (car_x_ - goal_x_) +
                               (car_y_ - goal_y_) * (car_y_ - goal_y_);
    
    // main RRT loop - build tree and check for goal condition
    for (int i = 0; i < max_iterations_; ++i) {
        std::vector<double> sample = this->sample();
        if (sample.empty()) {
            continue; // failed to sample a valid point, try again
        }

        int nearest_idx = this->nearest(tree, sample);
        RRT_Node new_node = this->steer(tree[nearest_idx], sample);

        /// basic RRT - connect to nearest node, long as no collision
        // if (this->check_collision(tree[nearest_idx], new_node)) {
        //     continue; // collision detected, discard this node and try again
        // }
        // new_node.parent = nearest_idx; // set parent index
        // tree.push_back(new_node); // add new node to tree

        // if (this->is_goal(new_node, goal_x_, goal_y_)) {
        //     found_path = this->find_path(tree, new_node);
        //     path_found = true;
        //     break; // path found, exit loop
        // }

        // RRT* - choose best parent from nearby nodes, still no collisions allowed
        std::vector<int> near_nodes = this->near(tree, new_node);
        if (std::find(near_nodes.begin(), near_nodes.end(), nearest_idx) == near_nodes.end()) {
            near_nodes.push_back(nearest_idx);
        }

        int best_parent = -1;
        double best_parent_cost = std::numeric_limits<double>::infinity();
        
        for (int idx : near_nodes) {
            // skip invalid indices
            if (idx < 0 || idx >= static_cast<int>(tree.size())) {
                continue;
            }
            // skip if theres a collision between candidate parent and new node
            if (this->check_collision(tree[idx], new_node)) {
                continue;
            }

            // cost to reach new node through parent candidate
            const double candidate_cost = this->cost(tree, tree[idx]) + this->line_cost(tree[idx], new_node);
            if (candidate_cost < best_parent_cost) {
                best_parent_cost = candidate_cost;
                best_parent = idx;
            }
        }

        if (best_parent == -1) {
            continue; // no collision-free way to connect
        }

        // add new node with best parent to tree
        new_node.parent = best_parent;
        new_node.cost = best_parent_cost;
        tree.push_back(new_node);
        const int new_idx = static_cast<int>(tree.size()) - 1;

        // check if new node is closer to tgt pt than previous best
        const double goal_dx = tree[new_idx].x - goal_x_;
        const double goal_dy = tree[new_idx].y - goal_y_;
        const double goal_dist_sq = goal_dx * goal_dx + goal_dy * goal_dy;

        if (goal_dist_sq < best_goal_dist_sq) {
            best_goal_dist_sq = goal_dist_sq;
            best_goal_idx = new_idx;
        }

        // rewire step  - improve neighbors if going through new node is cheaper
        const double new_node_cost = tree[new_idx].cost;
        for (int idx : near_nodes) {
            if (idx < 0 || idx >= new_idx || idx == best_parent) {
                continue;
            }
            if (this->check_collision(tree[new_idx], tree[idx])) {
                continue;
            }

            // cost to reach neighbor throuhg new node
            const double current_cost = this->cost(tree, tree[idx]);
            const double rewired_cost = new_node_cost + this->line_cost(tree[new_idx], tree[idx]);
            if (rewired_cost + 1e-6 < current_cost) {
                tree[idx].parent = new_idx;
                tree[idx].cost = rewired_cost;
            }
        }

        // check if new node reaches tgt pt
        if (this->is_goal(tree[new_idx], goal_x_, goal_y_)) {
            found_path = this->find_path(tree, tree[new_idx]);
            path_found = true;
            break; // path found, exit loop
        }
    }

    // push partial paths if goal isnt found, decent fallback for viz
    if (!path_found && tree.size() > 1 && best_goal_idx >= 0 && best_goal_idx < static_cast<int>(tree.size())) {
        found_path = this->find_path(tree, tree[best_goal_idx]);
        path_found = !found_path.empty();
    }

    // publish path msg for viz
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = pose_msg->header.stamp;
    path_msg.header.frame_id = "map";
    if (path_found) {
        // find_path returns goal->...->root; publish root->...->goal for visualization.
        for (auto it = found_path.rbegin(); it != found_path.rend(); ++it) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = it->x;
            pose.pose.position.y = it->y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
    }
    path_publisher_->publish(path_msg);

    // tree viz
    ++plan_cycle_count_;
    if (plan_cycle_count_ % tree_publish_stride_ == 0) {
        visualization_msgs::msg::Marker tree_marker;
        tree_marker.header.frame_id = "map";
        tree_marker.header.stamp = pose_msg->header.stamp;
        tree_marker.ns = "rrt_tree_edges";
        tree_marker.id = 0;
        tree_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        tree_marker.action = visualization_msgs::msg::Marker::ADD;
        tree_marker.scale.x = 0.02;
        tree_marker.color.b = 1.0;
        tree_marker.color.a = 0.5;

        for (const auto & node : tree) {
            if (node.parent != -1) {
                // Edge from node to its parent
                geometry_msgs::msg::Point p1, p2;
                p1.x = node.x; p1.y = node.y;
                p2.x = tree[node.parent].x; p2.y = tree[node.parent].y;
                tree_marker.points.push_back(p1);
                tree_marker.points.push_back(p2);
            }
        }

        // publish tree marker array (drop?)
        visualization_msgs::msg::MarkerArray tree_ma;
        tree_ma.markers.push_back(tree_marker);
        rrt_tree_publisher_->publish(tree_ma);
    }
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    // define sampling region (matches occupancy grid bounds)
    std::uniform_real_distribution<double> dist_x(origin_x_, origin_x_ + GRID_SIZE);
    std::uniform_real_distribution<double> dist_y(origin_y_, origin_y_ + GRID_SIZE);

    // bias sampling towards goal point to help convergence
    std::uniform_real_distribution<double> bias_dist(0.0, 1.0);
    double goal_prob = 0.35; // stronger goal bias improves consistency

    int attempts = 0;
    while (sampled_point.empty() && attempts < 250) {
        attempts++;

        double x, y;

        if (bias_dist(gen) < goal_prob) {
            x = goal_x_; // bias towards goal
            y = goal_y_;
        } else {
            x = dist_x(gen);
            y = dist_y(gen);
        }

        // convert world coordinates to grid indices
        int col = static_cast<int>((x - origin_x_) / GRID_RESOLUTION);
        int row = static_cast<int>((y - origin_y_) / GRID_RESOLUTION);

        // check if sampled point is in free space
        if (col >= 0 && col < GRID_CELLS && row >= 0 && row < GRID_CELLS) {
            if (is_cell_free(row, col)) {
                sampled_point = {x, y};
                break;
            }
        }
    }
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < tree.size(); ++i) {
        double dx = tree[i].x - sampled_point[0];
        double dy = tree[i].y - sampled_point[1];
        double dist = dx * dx + dy * dy; // squared distance, faster

        if (dist < min_dist_sq) {
            min_dist_sq = dist;
            nearest_node = i;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;

    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;
    double dist = dx * dx + dy * dy;

    // if the sample point is within max expand dist, use the point directly
    if (dist < max_expansion_dist_ * max_expansion_dist_) {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    } 
    else // take a step toward sample point of size max_expansion_dist
    {
        double theta = std::atan2(dy, dx);
        new_node.x = nearest_node.x + max_expansion_dist_ * std::cos(theta);
        new_node.y = nearest_node.y + max_expansion_dist_ * std::sin(theta);
    }    
    
    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;

    // convert world coords to grid idxs
    int x0 = static_cast<int>((nearest_node.x - origin_x_) / GRID_RESOLUTION);
    int y0 = static_cast<int>((nearest_node.y - origin_y_) / GRID_RESOLUTION);
    int x1 = static_cast<int>((new_node.x - origin_x_) / GRID_RESOLUTION);
    int y1 = static_cast<int>((new_node.y - origin_y_) / GRID_RESOLUTION);
    
    // use Bresenham line alg to check cells along a line for occupancy
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (!is_cell_free(y0, x0)) {
            collision = true; // collision / occupied cell
            break;
        }
        if (x0 == x1 && y0 == y1) {
            break; // reached end point of line
        }

        // step along the line
        int err2 = 2 * err; // keep everyting in ints
        if (err2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (err2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    double dx = latest_added_node.x - goal_x;
    double dy = latest_added_node.y - goal_y;
    double dist = dx * dx + dy * dy; // squared distance

    if (dist < goal_threshold_ * goal_threshold_) {
        close_enough = true;
    }

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    
    // start from goal node, work backbackwards to root
    RRT_Node current_node = latest_added_node;

    while (true) {
        found_path.push_back(current_node);
        
        if (current_node.parent == -1) {
            break; // reached root node (car position)
        }
        current_node = tree[current_node.parent]; // move to parent node
    }
    return found_path;
}

// RRT* methods ////////////////////////////////////////////////////////////
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cumulative cost from root to a node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current RRT tree
    //   node (RRT_Node): the node to calculate the cost for
    // Returns:
    //   total_cost (double): the cumulative cost from root to the node

    double total_cost = 0.0;
    RRT_Node current = node;
    int depth_guard = 0;

    // traverse to root, summing edge costs
    while (current.parent != -1) {
        const int parent_idx = current.parent;
        if (parent_idx < 0 || parent_idx >= static_cast<int>(tree.size())) {
            return std::numeric_limits<double>::infinity();
        }

        total_cost += this->line_cost(current, tree[parent_idx]);
        current = tree[parent_idx];
        ++depth_guard; 

        // prevent infinite loop if tree is bad
        if (depth_guard > static_cast<int>(tree.size())) {
            return std::numeric_limits<double>::infinity();
        }
    }

    return total_cost;
}


double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double dx = n1.x - n2.x;
    double dy = n1.y - n2.y;
    double cost = std::sqrt(dx * dx + dy * dy); // Euclidean distance

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;

    double r_sq = neighborhood_radius_ * neighborhood_radius_;

    for (size_t i = 0; i < tree.size(); ++i) {
        double dx = tree[i].x - node.x;
        double dy = tree[i].y - node.y;
        double dist = dx * dx + dy * dy; // squared distance

        if (dist <= r_sq) {
            neighborhood.push_back(static_cast<int>(i));
        }
    }

    return neighborhood;
}
