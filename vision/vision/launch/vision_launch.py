from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='vision_node',
            name='img_publisher'
        ),
        Node(
            package='vision',
            executable='img_subscriber',
            name='img_subscriber'
        ),
        Node(
            package='vision',
            executable='fake_task_selector',
            name='fake_task_selector',
            )
    ])