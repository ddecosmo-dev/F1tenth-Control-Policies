from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    model_name_arg = DeclareLaunchArgument('model_name', default_value='LSTM_1B_128D', description='Model name to load from config')
    ld.add_action(model_name_arg)

    node = Node(
        package='mapless_lstm_racer',
        executable='lstm_drive',
        name='lstm_drive_node',
        output='screen',
        parameters=[{'model_name': LaunchConfiguration('model_name')}],
    )

    ld.add_action(node)
    return ld
