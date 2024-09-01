import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the model path as a launch argument
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',  # Set a default path or make it mandatory by removing `default_value`
        description='Path to the YOLO model file'
    )

    # Log the model path to ensure it's being passed correctly
    log_model_path = LogInfo(msg=LaunchConfiguration('model_path'))

    model_server_node = Node(
        package='perception',  # perception
        executable='model_server',  
        name='model_server',
        output='screen',
        parameters=[{'model_path': LaunchConfiguration('model_path')}]
    )

    return LaunchDescription([
        model_path_arg,
        log_model_path,
        model_server_node
    ])
