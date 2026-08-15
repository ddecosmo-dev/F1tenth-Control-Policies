import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import os

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')
        
        # 1. Create Subscription
        self.log_sub = self.create_subscription(
            PointStamped, 
            '/clicked_point', 
            self.waypoint_callback, 
            10)
        
        # 2. FIXED PATH: This saves directly to your HOME folder
        self.csv_path = os.path.expanduser('~/waypoints.csv')
        
        # Ensure directory exists (not strictly needed for home, but good practice)
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

    def waypoint_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.get_logger().info(f"Logged: x={x:.2f}, y={y:.2f}")
        
        # Write and FORCE save to disk immediately
        self.csv_writer.writerow([x, y])
        self.csv_file.flush() 

    def close_csv(self):
        self.get_logger().info("Closing CSV file...")
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()