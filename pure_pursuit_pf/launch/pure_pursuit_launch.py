from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main Pure Pursuit Controller Node
        Node(
            package='pure_pursuit_pf',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            # parameters=[{'waypoint_file': '/your/path/here.csv'}] # Uncomment if you switch to parameters later
        ),

        # Waypoint Visualizer Node
        Node(
            package='pure_pursuit_pf',
            executable='waypoint_viz_node',
            name='waypoint_viz_node',
            output='screen'
        )
    ])