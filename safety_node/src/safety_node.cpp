#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <ranges>

#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        odo_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, _1));

        scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Safety::scan_callback, this, _1));

        //define publisher for ackermann speed message
        //return to this later, checknodes for correct things!
        speed_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

    }

private:
    double x_speed {0.0};
    double y_speed {0.0};

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        //Update x and y speed from Odo
        x_speed = msg->twist.twist.linear.x;
        y_speed = msg->twist.twist.linear.y;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        //Get scan info 
        std::vector<float> range_vector = scan_msg->ranges;
        std::vector<float> iTTC_vector(range_vector.size());

        //angle info
        float angle_min = scan_msg->angle_min;
        float angle_max = scan_msg->angle_max;
        float angle_increment  = scan_msg->angle_increment;

        // iTTC calculation
        for (size_t i = 0; i < range_vector.size(); i++) {
            //calculate numerator and denominator of equation
            double num = range_vector[i];
            double den = x_speed * std::cos(angle_min + angle_increment * i);
            den = std::max(den,0.0);

            //calculate iTTCs and handle nans and infs
            if (den > 0.0001) {
                iTTC_vector[i] = num/den;
            }
            else {
                iTTC_vector[i] = std::numeric_limits<double>::infinity();
            }

            // if (!std::isnan(den) || (!std::isinf(den))) {
            //     iTTC_vector[i] = num/den;
            // }
            // else {
            //     iTTC_vector[i] = 100.0;
            // }
        }
        
        /// TODO: 
        // Check if we need more than 1 check, 
        // consider error hadnling
        
        auto min_iTTC_index = std::min_element(iTTC_vector.begin(), iTTC_vector.end());
        double iTTC_loop = *min_iTTC_index;
        
        double min_iTTC{1};

        if (iTTC_loop <= min_iTTC) {
            auto message = ackermann_msgs::msg::AckermannDriveStamped();
            message.header.stamp = this->now();
            message.drive.speed = 0.0;

            speed_publisher->publish(message);
            RCLCPP_INFO(this->get_logger(), "AUTO STOP ON!!!!");
        }

        RCLCPP_INFO(this->get_logger(), "Incoming Values: X_speed:%.2f, Min ITTC %.2f, Angle Increment:%.2f",
        x_speed,iTTC_loop,angle_increment);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr speed_publisher;

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}