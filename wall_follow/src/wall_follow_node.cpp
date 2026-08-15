#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&WallFollow::scan_callback, this, _1));

        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    // PID CONTROL PARAMS
    /// TODO: TUNE PID, add to function when completed
    const double kp = 1;
    const double kd = 0.15;
    const double ki = 0.0;

    //get error for the loop
    /// ADJUST: desired dist from wall, take a look at the map in sim (use measure tool)
    const double d_desired = 1.0; //start with 1m or 
    const double l_dist = 0.5; //start with 1m considering calculating with max speed and publishing rate
    double angle_rate = 3; 

    static constexpr double PI = 3.14159;

    double servo_offset = 0.0;
    double prev_error = 0.0;
    rclcpp::Time prev_timestamp = this->now();

    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers


    //add inputs for min and increment 
    double get_range(const std::vector<float> &range_vector, const double angle, const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_vector: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR
            msg: laserscan msg, for angle info

        Returns:
            range: range measurement in meters at the given angle
        */

        //angle info
        const float angle_min = scan_msg->angle_min;
        const float angle_max = scan_msg->angle_max;
        const float angle_increment  = scan_msg->angle_increment;

        //calculate index of range based off angle and angle increment 
        const int idx = static_cast<int>((angle - angle_min) / (angle_increment));
        //int idx = (angle + max_angle) / (angle_increment)

        float range = range_vector[idx];

        //Inf and Nan handling
        if (!std::isfinite(range)) {
            return scan_msg->range_max; 
        }

        return range;
    }

    double get_error(const std::vector<float> &range_vector, const double d_desired, const double l_dist, const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_da: single range array from the LiDAR
            d_desired: desired distance to the wall
            l_dist: lookahead distance

        Returns:
            error: calculated error
        */

        //set angles, using leftfacing to move counterclockwise 
        const double angle_a = PI / 4; // 65 degrees
        const double angle_b = PI / 2;  // 
        const double theta = angle_b - angle_a;

        //get ranges at a and b
        const double range_a = get_range(range_vector, angle_a, scan_msg);
        const double range_b = get_range(range_vector, angle_b, scan_msg);

        //calculate angle alpha
        const double alpha = std::atan2((range_a * std::cos(theta) - range_b),(range_a * std::sin(theta)));

        //find Dt
        const double Dt = range_b * std::cos(alpha);

        //find Dt_1
        const double Dt_1 = Dt + l_dist * std::sin(alpha);

        //Calculate and return error 
        // return d_desired - Dt_1;
        return Dt_1 - d_desired;
    }

    //do we need velocity here?
    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

        //get dt
        rclcpp::Time current_timestamp = this->now();
        double dt = (current_timestamp - prev_timestamp).seconds();
        if (dt <= 0) return;

        //reset time for next iteration
        prev_timestamp = current_timestamp;

        //PID calculation of steering angle
        integral += error *dt;
        double angle = kp * error + ki * (integral) + kd * ((error - prev_error) / dt);

        //clamp output angle values between +/- 0.4 rad
        angle = std::max(-0.4, std::min(0.4,angle));

        //velocity control
        if (std::abs(angle) >= 0 && std::abs(angle) < PI/18) {
            velocity = 3.0;
        }
        else if (std::abs(angle) >= PI/18 && std::abs(angle) < PI/12) {
            velocity = 2.0;
        }
        else if (std::abs(angle) >= PI/12 && std::abs(angle) < PI/9) {
            velocity = 1.0;
        }
        else {
            velocity = 0.5;
        }

        /// TODO: 
        //limits to steering
        //steering angle rate calculation (add later if needed)
        //add to message if time

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "drive";
        drive_msg.drive.speed = velocity; 
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.steering_angle_velocity = angle_rate;

        drive_publisher->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "Current Velocity: %.2f, Current Error: %.2f, Steering Angle: %.2f",
        velocity,error,angle);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

        //Get scan info 
        const std::vector<float> &range_vector = scan_msg->ranges;

        //calculate iter error
        double error = get_error(range_vector, d_desired, l_dist, scan_msg);

        //PID calc
        double velocity = 0.5;
        pid_control(error,velocity);

        prev_error = error;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}