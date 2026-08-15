#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/parameter_client.hpp"

using namespace std;
using std::placeholders::_1;

class WaypointViz : public rclcpp::Node 
{

public:
    WaypointViz() : Node("waypoint_viz_node")
    {
        //create sub for desired node
        waypoint_subscriber = this->create_subscription<std_msgs::msg::Int32>("/active_waypoint", 10, 
            std::bind(&WaypointViz::marker_callback, this, _1));

        //waypoint publishers
        //array
        array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/array_markers",10);
        //active node
        active_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/active_marker",10);

        //Load CSV for visualization
        load_waypoint_csv(csv_path);

        //get parameters from pure_pursuit (search window)
        // parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "pure_pursuit_node");

        // 3. Perform the logic inside the constructor
        // int target_window = 5;
        // if (parameters_client->wait_for_service(std::chrono::seconds(1))) {
        //     target_window = parameters_client->get_parameter<int>("target_window", 5);
        //     RCLCPP_INFO(this->get_logger(), "Successfully fetched target_window: %d", target_window);
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "Sim node not found after 1s. Defaulting window to 5.");
        //     target_window = 5;
        // }
    }

private:
    //global variables
    //ssv
    std::string csv_path = "/home/f1humble/sim_ws/src/pure_pursuit_pf/config/waypoints.csv";

    //waypoint setup
    struct waypoints {
        double x;
        double y;
        //double v, if we want later;
    };

    std::vector<waypoints> global_waypoints;
    bool csv_loaded = false;

    //target window setup
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
    int target_window;
    
    //________________________________________________

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

    void search_markers(int target_idx) {
        // if (parameters_client->service_is_ready()) {
        // target_window = parameters_client->get_parameter<int>("target_window", 5);
        // }

        int target_window = 5;

        int n_size = static_cast<int>(global_waypoints.size());
        int id_counter = 0;

        if (!csv_loaded || global_waypoints.empty()) return;
        visualization_msgs::msg::MarkerArray search_array;

        for (int i = -target_window; i <= target_window; i++) {
            int curr_idx = (target_idx + i + n_size) % n_size;
            auto& wp = global_waypoints[curr_idx];

            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();

            m.ns = "search_nodes";
            m.id = id_counter++;

            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            
            //coordinates
            m.pose.position.x = wp.x;
            m.pose.position.y = wp.y;
            m.pose.position.z = 0.0;

            //orinetation
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = 0.0;
            m.pose.orientation.w = 1.0;

            //size
            m.scale.x = 0.2;
            m.scale.y = 0.2;
            m.scale.z = 0.2;

            //color
            m.color.r = 0.0f;
            m.color.g = 0.0f;
            m.color.b = 1.0f;
            m.color.a = 1.0f;

            //add to array message
            search_array.markers.push_back(m);
        }
        array_publisher->publish(search_array);
    }

    void array_markers() {
        if (!csv_loaded || global_waypoints.empty()) return;

        visualization_msgs::msg::MarkerArray marker_array;
        int id_counter = 0;

        for (const auto& wp : global_waypoints) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = this->now();
            m.ns = "track_nodes";
            m.id = id_counter++;

            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            
            //coordinates
            m.pose.position.x = wp.x;
            m.pose.position.y = wp.y;
            m.pose.position.z = 0.0;

            //orinetation
            m.pose.orientation.x = 0.0;
            m.pose.orientation.y = 0.0;
            m.pose.orientation.z = 0.0;
            m.pose.orientation.w = 1.0;

            //size
            m.scale.x = 0.15;
            m.scale.y = 0.15;
            m.scale.z = 0.15;

            //color
            m.color.r = 1.0f;
            m.color.g = 0.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;

            //add to array message
            marker_array.markers.push_back(m);
        }
        array_publisher->publish(marker_array);
    }

    void active_marker(int target_idx) {
        if (target_idx < 0 || target_idx >= (int)global_waypoints.size()) return;

        //get active point
        const waypoints wp_t = global_waypoints[target_idx];

        //create message type
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = this->now();
        m.ns = "track_nodes";
        m.id = 9999;

        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;

        //create message
        //coordinates
        m.pose.position.x = wp_t.x;
        m.pose.position.y = wp_t.y;
        m.pose.position.z = 0.0;

        //orinetation
        m.pose.orientation.x = 0.0;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = 1.0;

        //size
        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;

        //color
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        m.color.a = 1.0f;

        //publish message
        active_publisher->publish(m);
    }

    void marker_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        //call the message
        //save message as normal int
        int target_idx = msg->data;

        //call array
        array_markers();

        //call search
        search_markers(target_idx);

        //call active
        active_marker(target_idx);
    } 

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_subscriber;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr array_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr active_publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointViz>());
    rclcpp::shutdown();
    return 0;
}



