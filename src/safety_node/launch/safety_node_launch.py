#Based off ROS tutorials for making launch files and params.yaml files

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    #gym package
    gym_pkg_share = get_package_share_directory('f1tenth_gym_ros')
    gym_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gym_pkg_share, 'launch', 'gym_bridge_launch.py'))
    )

    #talker node
    safety_node = Node (
        package = 'safety_node',
        name = 'safety_node',
        executable = 'safety_node',
        output='screen',
    )


    ld.add_action(safety_node)
    ld.add_action(gym_launch)
    return ld