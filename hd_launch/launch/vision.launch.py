from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python import get_package_share_directory


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    # Declare the launch argument for the camera type
    # Define the LaunchConfiguration for the camera type
    hd_topic_names_file = get_package_file('custom_msg', 'config/hd_interface_names.yaml')

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('camera'),
                'launch/camera_node.launch.py'))
    )

    return LaunchDescription(
        [

            # Camera node with the parameter set from the launch argument
           launch_include,
            Node(package="perception", executable="perception_node", name="perception"),
            Node(package="vision", executable="img_subscriber", name="vision"),
            Node(package="perception", executable="gui_node", name="perception"),
        ]
    )
