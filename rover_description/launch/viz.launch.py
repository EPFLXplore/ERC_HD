import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
 
 
 
def generate_launch_description():
 
    
    package_name='rover_description' 
    rviz_config_path = os.path.join(get_package_share_directory(package_name), 'config', 'view.rviz')
    

  
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    

    return LaunchDescription([
        rviz_node,
    ])
