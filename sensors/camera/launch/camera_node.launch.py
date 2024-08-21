from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera",
                executable="camera",
                name="camera",
                parameters=[
                    {"camera_type": "realsense_stereo"}
                ],  # Use 'mock_stereo' or 'mock_monocular'
            )
        ]
    )
