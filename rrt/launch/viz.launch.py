from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for RRT Visualization Node.
    
    Starts:
    - viz_node: Visualization marker aggregator for RViz
    """
    
    return LaunchDescription([
        Node(
            package='lab06_pkg',
            executable='viz_node',
            name='viz_node',
            output='screen',
        ),
    ])
