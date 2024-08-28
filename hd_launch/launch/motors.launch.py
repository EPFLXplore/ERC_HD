from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    operator_param = DeclareLaunchArgument(
        "operator",
        default_value="jean",
        description="Operator controlling the arm"
    )
    
    return LaunchDescription([
        operator_param,
        Node(package="ethercat_device_configurator", executable="motor_control"),
        Node(package="fake_components", executable="stats_tracker.py", parameters=[{"operator": LaunchConfiguration("operator")}])
    ])
