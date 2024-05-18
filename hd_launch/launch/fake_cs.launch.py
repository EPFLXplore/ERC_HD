from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d'],
    )
    
    return LaunchDescription(
        [
            Node(
                package="vision",
                executable="img_subscriber",
            ),
            Node(
                package="fake_components",
                executable="fake_cs_gamepad.py",
            ),
            #rviz
        ]
    )