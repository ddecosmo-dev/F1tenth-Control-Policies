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


    #talker node
    gap_follow = Node (
        package = 'gap_follow',
        name = 'gap_follow',
        executable = 'reactive_node',
        output='screen',
    )

    ld.add_action(gap_follow)
    return ld