
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():


    return LaunchDescription([
   
         launch_ros.actions.Node(
            package='rover_description',
            executable='hd_sub.py',
            output='screen'
            
        ) 


        ])



        
