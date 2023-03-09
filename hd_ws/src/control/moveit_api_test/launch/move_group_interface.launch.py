from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("astra").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_test",
        package="moveit_api_test",
        executable="move_group_interface_test",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])