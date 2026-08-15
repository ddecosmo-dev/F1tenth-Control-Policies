from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    output_file_arg = DeclareLaunchArgument(
        'output_file', default_value='lstm_racer_log.npz', description='Base name for the logger NPZ output file (stored in package logs folder)'
    )
    output_csv_arg = DeclareLaunchArgument('output_csv', default_value='lstm_racer_log.npz', description='Alias for output_file for compatibility')
    model_name_arg = DeclareLaunchArgument('model_name', default_value='', description='Model name to include in the log filename')
    goal_center_x_arg = DeclareLaunchArgument('goal_center_x', default_value='0.0', description='Goal center x coordinate')
    goal_center_y_arg = DeclareLaunchArgument('goal_center_y', default_value='0.0', description='Goal center y coordinate')
    goal_offset_x_arg = DeclareLaunchArgument('goal_offset_x', default_value='0.0', description='Goal offset x relative to start pose')
    goal_offset_y_arg = DeclareLaunchArgument('goal_offset_y', default_value='0.0', description='Goal offset y relative to start pose')
    use_goal_offset_arg = DeclareLaunchArgument('use_goal_offset', default_value='true', description='Use goal offset relative to start pose')
    goal_radius_arg = DeclareLaunchArgument('goal_radius', default_value='1.0', description='Goal radius for completion')
    start_radius_arg = DeclareLaunchArgument('start_radius', default_value='1.0', description='Start region radius')
    use_start_pose_as_start_center_arg = DeclareLaunchArgument('use_start_pose_as_start_center', default_value='true', description='Use initial odom pose as the start center')
    collision_topic_arg = DeclareLaunchArgument('collision_topic', default_value='', description='Bool topic that signals collisions')

    ld.add_action(output_file_arg)
    ld.add_action(output_csv_arg)
    ld.add_action(model_name_arg)
    ld.add_action(goal_center_x_arg)
    ld.add_action(goal_center_y_arg)
    ld.add_action(goal_offset_x_arg)
    ld.add_action(goal_offset_y_arg)
    ld.add_action(use_goal_offset_arg)
    ld.add_action(goal_radius_arg)
    ld.add_action(start_radius_arg)
    ld.add_action(use_start_pose_as_start_center_arg)
    ld.add_action(collision_topic_arg)

    logger_node = Node(
        package='mapless_lstm_racer',
        executable='lstm_logger',
        name='lstm_logger_node',
        output='screen',
        parameters=[
            {
                'output_file': LaunchConfiguration('output_file'),
                'output_csv': LaunchConfiguration('output_csv'),
                'model_name': LaunchConfiguration('model_name'),
                'goal_center_x': LaunchConfiguration('goal_center_x'),
                'goal_center_y': LaunchConfiguration('goal_center_y'),
                'goal_offset_x': LaunchConfiguration('goal_offset_x'),
                'goal_offset_y': LaunchConfiguration('goal_offset_y'),
                'use_goal_offset': LaunchConfiguration('use_goal_offset'),
                'goal_radius': LaunchConfiguration('goal_radius'),
                'start_radius': LaunchConfiguration('start_radius'),
                'use_start_pose_as_start_center': LaunchConfiguration('use_start_pose_as_start_center'),
                'collision_topic': LaunchConfiguration('collision_topic'),
            },
        ],
    )

    ld.add_action(logger_node)
    return ld
