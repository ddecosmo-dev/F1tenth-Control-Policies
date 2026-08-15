#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));

        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    //topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    //global variables, alterable parameter
    const float r_car = 0.3; 

    const float d_threshold = 10.0;

    double angle_rate = 3; 
    double prev_steering_angle = 0.0;
    
    const double PI = 3.141592653589793;

    void preprocess_lidar(std::vector<float> &ranges)
    {   
        // nan and inf checks
        for (float &range : ranges) {
            if (std::isinf(range) || std::isnan(range)) range = 0.0;
            // else if (range > d_threshold) range = d_bubble_threshold;
        }

        std::vector<float> smoothed_ranges = ranges;
        const int preprocess_window = 5;
        const int half_preprocess = preprocess_window / 2;
        int n = static_cast<int>(ranges.size());

        // Use int i to match the casted bounds
        for (int i = half_preprocess; i < n - half_preprocess; i++) {
            float sum = ranges[i-2] + ranges[i-1] + ranges[i] + ranges[i+1] + ranges[i+2];
            smoothed_ranges[i] = sum / preprocess_window;
        }

        ranges = smoothed_ranges;
    }

    //disparity extender function
    void disparity_extender(std::vector<float> & ranges, float angle_increment) {
    const int n = static_cast<int>(ranges.size());
    std::vector<float> disparity_ranges = ranges; 

    for (int i = 0; i < n - 1; ++i) {
        float dist_curr = ranges[i];
        float dist_next = ranges[i + 1];
        float disparity = std::abs(dist_curr - dist_next);

        // Disparity threshold 
        if (disparity > 0.3f) {
            // Use the closer point to calculate the "shadow" width
            float min_dist = std::max(std::min(dist_curr, dist_next), 0.1f);
            
            // Calculate how many lidar beams the car's width covers at this distance
            float angle_width_rad = std::atan2(r_car, min_dist);
            int overwrite_count = static_cast<int>(std::ceil(angle_width_rad / angle_increment));

            if (dist_curr < dist_next) {
                // Wall is on the left, extend the shadow to the right (farther points)
                for (int k = 1; k <= overwrite_count; ++k) {
                    int target_idx = i + k;
                    if (target_idx < n) {
                        disparity_ranges[target_idx] = std::min(disparity_ranges[target_idx], min_dist);
                    }
                }
            } else {
                // Wall is on the right, extend the shadow to the left (farther points)
                for (int k = 0; k < overwrite_count; ++k) {
                    int target_idx = i - k;
                    if (target_idx >= 0) {
                        disparity_ranges[target_idx] = std::min(disparity_ranges[target_idx], min_dist);
                    }
                }
            }
        }
    }
    // Move the modified ranges back to the original vector
    ranges = std::move(disparity_ranges);
    }

    //Old, used for detecting only closest obstacle.
    // //clamp outputs 
    void find_bubble(std::vector<float> &ranges, int start_idx, int end_idx,float angle_increment) {
        // function to identify closest obstacle and set values to zero closest to it 

        float min_dist = 100.0;
        int min_idx = 0;
        
        //find minimum point get idx and range
        //come back and get it to look only at angles in front (+90 to -90)
        for (int i = start_idx; i < end_idx; i++) {
            if (ranges[i] == 0.0) { continue; }
            else if (ranges[i] < min_dist) {
                min_dist = ranges[i];
                min_idx = i;
            }
        }
        
        //find angle theta
        float ratio = r_car / ranges[min_idx];
        if (ratio > 1.0) ratio = 1.0; 
        float theta = std::asin(ratio);

        //find radius of idx
        int rad_idx = static_cast<int>(theta / angle_increment);

        //set bubble to zero
        int bubble_start = std::max(0, min_idx - rad_idx);
        int bubble_end = std::min(static_cast<int>(ranges.size() - 1),(min_idx + rad_idx));

        for (int j = bubble_start; j <= bubble_end; j++) {
            ranges[j] = 0.0;
        }

    }


    int find_gap_idx(std::vector<float> &ranges, int center_idx)
    {   

        int best_len{0};
        int best_start_idx{0};
        int best_end_idx{0};
        
        int current_len{0};
        int current_start_idx{0};

        bool gap_check {false};

        //iterate through the entire range
        for (size_t i = 0; i < ranges.size(); i++) {
            if (ranges[i] == 0) {
                if (current_len > best_len){
                    best_len = current_len;
                    best_start_idx = current_start_idx;
                    best_end_idx = i - 1;
                }
                current_len = 0;
                current_start_idx = 0;
                gap_check = false; 
            }

            else if (ranges[i] > 0) {
                if (!gap_check) { 
                    current_start_idx = i;
                    gap_check = true;  
                }
                current_len++; 
            }
        }

        if (current_len > best_len) {
        best_len = current_len;
        best_start_idx = current_start_idx;
        best_end_idx = ranges.size() - 1;
        }

        //find central index, take center to be the best

        float max_dist = -1.0;
        int max_idx = best_start_idx; // Default fallback

        //max dist
        for (int i = best_start_idx; i <= best_end_idx; i++) {
            if (ranges[i] > max_dist) {
                max_dist = ranges[i];
                max_idx = i;
            }
            
            //push idx closer to center if ties
        else if (std::abs(ranges[i] - max_dist) < 1e-3) {
            if (std::abs(i - center_idx) < std::abs(max_idx - center_idx)) {
                max_idx = i;
            }
        }
        }

        // int idx = (best_start_idx + best_end_idx) / 2;
        int idx = max_idx;

        return idx;
    }

    void reactive_control(int idx, float angle_min, float angle_increment) {
        //take the gap index and convert to a steering angle
        //handle velocity control
        //publish to /drive topic

        double target_angle = angle_min + angle_increment * idx;    
        target_angle = std::max(-0.4, std::min(0.4, target_angle));

        //smooth steering slightly
        double alpha = 0.5; 
        double smoothed_angle = (alpha * target_angle) + (1.0 - alpha) * prev_steering_angle;
        prev_steering_angle = smoothed_angle;
        double angle = smoothed_angle;

        //clamp output angle values between +/- 0.4 rad
        angle = std::max(-0.4, std::min(0.4,angle));

        //velocity control
        double velocity{0.5};

        if (std::abs(angle) >= 0 && std::abs(angle) < PI/18) {
            velocity = 1.5;
        }
        else if (std::abs(angle) >= PI/18 && std::abs(angle) < PI/12) {
            velocity = 1.5;
        }
        else if (std::abs(angle) >= PI/12 && std::abs(angle) < 0.38) {
            velocity = 1;
        }
        else {
            velocity = 0.5;
        }

        //Then publish message
        //may want to update the logger with additional info
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "drive";
        drive_msg.drive.speed = velocity; 
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.steering_angle_velocity = angle_rate;

        drive_publisher->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "Current Velocity: %.2f, Steering Angle: %.2f",
        velocity,angle);
    }

    /// TODO: 
    /// implement disparity extender
    /// how to get steering angle to cheange less, carry over information across iters


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        
        //Get scan info 
        std::vector<float> range_vector = scan_msg->ranges;

        const float angle_min = scan_msg->angle_min;
        const float angle_increment  = scan_msg->angle_increment;

        //Set lidar limits +/- 90 degrees
        float half_fov = PI / 2.0; // 90 degrees
        int center_idx = range_vector.size() / 2;
        int offset = static_cast<int>(half_fov / angle_increment);

        int start_idx = std::max(0, center_idx - offset);
        int end_idx = std::min(static_cast<int>(range_vector.size() - 1), center_idx + offset);

        //preprocess lidar
        preprocess_lidar(range_vector);

        disparity_extender(range_vector, angle_increment);

        for (int i = 0; i < start_idx; ++i) range_vector[i] = 0.0;
        for (int i = end_idx + 1; i < static_cast<int>(range_vector.size()); ++i) range_vector[i] = 0.0;

        //perform bubble operations
        find_bubble(range_vector, start_idx, end_idx, angle_increment);

        //find gaps and target idx. 
        const int gap_idx {find_gap_idx(range_vector, center_idx)};

        //apply control to system
        reactive_control(gap_idx, angle_min,angle_increment);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}