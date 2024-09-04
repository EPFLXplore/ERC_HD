import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    hd_topic_names_file = get_package_file('custom_msg', 'config/hd_interface_names.yaml')

    # Declare the model path as a launch argument
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',  # Set a default path or make it mandatory by removing `default_value`
        description='Path to the YOLO model file'
    )

    # Log the model path to ensure it's being passed correctly
    # log_model_path = LogInfo(msg=LaunchConfiguration('model_path'))

    model_node = Node(
        package='perception',  # perception
        executable='model_node',  
        name='model_node',
        output='screen',
        parameters=[hd_topic_names_file]  # {'model_path': LaunchConfiguration('model_path')},
    )

    return LaunchDescription([
        model_path_arg,
        # log_model_path,
        model_node
    ])
