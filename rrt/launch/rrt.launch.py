from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for RRT Motion Planning Lab.
    
    Starts:
    - rrt_node: RRT path planner with occupancy grid
    - pure_pursuit_node: Waypoint follower that publishes goal points
    
    Launch viz_node separately with: ros2 launch lab06_pkg viz.launch.py
    """
    
    return LaunchDescription([
        Node(
            package='lab06_pkg',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
        ),
        Node(
            package='lab06_pkg',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
        ),
    ])
