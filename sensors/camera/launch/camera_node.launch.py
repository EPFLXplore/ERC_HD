from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from rclpy.parameter import Parameter
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Declare a launch argument for the camera type with a default value
    camera_type_arg = DeclareLaunchArgument(
        "camera_type", 
        default_value=TextSubstitution(text="realsense_stereo"), 
        description="Type of camera to be used. Can be one of: realsense_stereo, oakd_stereo, monocular, mock_stereo, mock_monocular"
    )

    return LaunchDescription(
        [
            # Include the declared argument
            camera_type_arg,

            # Camera node with the parameter set from the launch argument
            Node(
                package="camera",
                executable="camera",
                name="camera",
                parameters=[
                    {"camera_type": LaunchConfiguration('camera_type')}
                ],
                output='screen'  # Ensuring the logs are visible on the screen
            ),
        ]
    )
