#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/int32.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


/// CHECK: include needed ROS msg type headers and libraries

using namespace std;
using std::placeholders::_1;

class PurePursuit : public rclcpp::Node
{

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        /// TODO: create ROS subscribers and publishers

        ///TODO: Localization pub subs
        //For sim: sub to ODO
        odo_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

        //publish to drive
        drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        //waypoint publisher for viz
        waypoint_publisher = this->create_publisher<std_msgs::msg::Int32>("/active_waypoint",10);

        //load csv data
        load_waypoint_csv(csv_path);

        //declare window vairable
        this->declare_parameter("target_window",5);
        target_window = this->get_parameter("target_window").as_int();
    }
    ~PurePursuit() {}

private:
    //global variables
    //geometry message
    geometry_msgs::msg::Pose current_pose;
    int prev_target_idx = 0;
    int target_window{}; //how many indices on each side to check when searching for next waypoint

    //csv path
    //path to csv file
    bool csv_loaded = false; 
    std::string csv_path = "/home/f1humble/sim_ws/src/pure_pursuit_sim/config/waypoints.csv";
    struct waypoints {
        double x;
        double y;
        //double v, if we want later;
    };

    std::vector<waypoints> global_waypoints;

    //state variables 
    const double L_threshold = 1.5;
    const double PI = 3.14159; 
    const double CAR_WHEELBASE = 0.33; //m

    void load_waypoint_csv(std::string csv_path) {

        std::ifstream file(csv_path);
        std::string line;

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;

            if (std::getline(ss,x_str,',') && std::getline(ss,y_str,',')) {
                waypoints wp;
                wp.x = std::stod(x_str);
                wp.y = std::stod(y_str);
                global_waypoints.push_back(wp);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints.", global_waypoints.size());
        csv_loaded = true; 
    }

    int target_waypoint_sim(std::vector<waypoints> &global_waypoints, int prev_target_idx, int target_window) {
    
        //takes odo, waypoints vector, past index, and search window
        //calculates the target waypoint for pure pursuit

        //find x and y coordinates of vehicle
        float x_car = current_pose.position.x;
        float y_car = current_pose.position.y;

        //find indices to search through based on the window
        int target_idx = prev_target_idx;
        int n_size = static_cast<int>(global_waypoints.size());
        double min_d = 1e10;

        for (int i = -target_window; i <= target_window; i++) {
            int curr_idx = (prev_target_idx + i + n_size) % n_size;

            //check ahead of the car
            tf2::Vector3 loop_pt = target_transform(curr_idx);

            if (loop_pt.x() > 0) {
                float dx = global_waypoints[curr_idx].x - x_car;
                float dy = global_waypoints[curr_idx].y - y_car;

                float d = std::sqrt(std::pow(dx,2)+std::pow(dy,2));
                if (d > L_threshold && d < min_d) {
                    min_d = d;
                    target_idx= curr_idx;
                }
            }
        }

        //publish target index for visualization
        std_msgs::msg::Int32 wp_msg;
        wp_msg.data = target_idx;
        waypoint_publisher->publish(wp_msg);
        RCLCPP_INFO(this->get_logger(),"Current Waypoint Target: %i",target_idx); 
        
        return target_idx;
    }

    tf2::Vector3 target_transform(int target_idx) {
        //takes goal point and ODO
        //transforms goal point to vehicle frame of refernce
        
        // EQ: P_local = Rt * (Pglobal - t)

        //t term
        double x_car = current_pose.position.x;
        double y_car = current_pose.position.y;
        double z_car = 0.0;

        tf2::Vector3 t_car(x_car,y_car,z_car);

        //P global term
        double x_target = global_waypoints[target_idx].x;
        double y_target = global_waypoints[target_idx].y;
        double z_target = 0.0;

        tf2::Vector3 p_global(x_target,y_target,z_target);

        //Rotation matrix
        //quaternion components
        double ox = current_pose.orientation.x;
        double oy = current_pose.orientation.y;
        double oz = current_pose.orientation.z;
        double ow = current_pose.orientation.w;

        tf2::Quaternion car_Q(ox,oy,oz,ow);

        tf2::Matrix3x3 rot_matrix(car_Q);
        tf2::Matrix3x3 rot_matrix_T = rot_matrix.transpose();

        tf2::Vector3 target_point_car(rot_matrix_T * (p_global - t_car));

        return target_point_car;
    }

    void pursuit_control(tf2::Vector3 target_point_car) {
        //takes updated goal and 
        //calculates and published steeering and velocity

        //1. Solve for curvature 
        double target_y = target_point_car.y(); 
        double gamma = (2 * target_y) / std::pow(L_threshold,2);

        //2. Solve for steering angle
        double angle = std::atan(gamma * CAR_WHEELBASE);
        
        //Clamp angle 
        angle = std::max(-0.4, std::min(0.4,angle));

        double velocity = 1.0;

        //3. Velocity Control
                if (std::abs(angle) >= 0 && std::abs(angle) < PI/18) {
            velocity = 2.5;
        }
        else if (std::abs(angle) >= PI/18 && std::abs(angle) < PI/12) {
            velocity = 2;
        }
        else if (std::abs(angle) >= PI/12 && std::abs(angle) < 0.38) {
            velocity = 1.5;
        }
        else {
            velocity = 1;
        }
        
        
        //update later!!!
        // velocity = 1.0;

        //4. Publish
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "drive";
        drive_msg.drive.speed = velocity; 
        drive_msg.drive.steering_angle = angle;
        //drive_msg.drive.steering_angle_velocity = angle_rate;

        drive_publisher->publish(drive_msg);
        RCLCPP_INFO(this->get_logger(), "Current Curvature: %.2f, Current Velocity: %.2f, Steering Angle: %.2f",
        gamma,velocity,angle);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // //load csv
        // if (csv_loaded == false){ 
        //     load_waypoint_csv(csv_path);
        // }
    
        //get geometry info     
        current_pose = msg->pose.pose; 
        
        //organize raw waypoint data
        //int num_waypoints = global_waypoints.size();

        //Find the target waypoint 
        int target_idx = target_waypoint_sim(global_waypoints,
            prev_target_idx, target_window);

        // const waypoints odo_target = global_waypoints[target_idx]; //query csv however 
        prev_target_idx = target_idx;
        
        tf2::Vector3 car_target = target_transform(target_idx);
        
        //find and publish steering and velocity 
        pursuit_control(car_target);        
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_subscription;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}