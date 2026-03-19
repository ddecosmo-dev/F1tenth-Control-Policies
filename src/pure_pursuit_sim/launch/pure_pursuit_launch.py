import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_rrt_dd')
    
    # Define the exact paths to your config and CSV files
    config_dir = os.path.join(pkg_dir, 'config')
    params_file = os.path.join(config_dir, 'params.yaml')
    waypoints_file = os.path.join(config_dir, 'waypoints.csv')
    velocity_file = os.path.join(config_dir, 'velocity.csv')
    #rviz_config_file = os.path.join(config_dir, 'rrt_config.rviz') # Make sure this file exists!

    # Create a dictionary for the CSV paths to pass as parameters
    csv_paths = {
        'waypoint_path': waypoints_file,
        'velocity_path': velocity_file
    }

    return LaunchDescription([
        # 1. RRT Main Node
        Node(
            package='f1tenth_rrt_dd',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
            parameters=[params_file, csv_paths] 
        ),

        # 2. RRT Visualizer Node
        Node(
            package='f1tenth_rrt_dd',
            executable='rrt_viz',
            name='rrt_viz_node',
            output='screen',
            parameters=[params_file, csv_paths] 
        ),

        # 3. Pure Pursuit Main Node
        Node(
            package='f1tenth_rrt_dd',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[params_file, csv_paths]
        ),

        # 4. Pure Pursuit Visualizer Node
        Node(
            package='f1tenth_rrt_dd',
            executable='pure_pursuit_viz', 
            name='pure_pursuit_viz_node',
            output='screen',
            parameters=[params_file, csv_paths]
        ),

        # # 5. RViz2 Node (Loads your saved config automatically)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # )
    ])