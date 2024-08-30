from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Declare the launch argument for the camera type
    # Define the LaunchConfiguration for the camera type

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
            Node(package='perception', executable='gui_node', name='gui_hd'),
        ]
    )
