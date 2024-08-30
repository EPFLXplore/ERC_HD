
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():


    return LaunchDescription([
   
         launch_ros.actions.Node(
            package='rover_description',
            executable='wheel_vel_sub.py',
            output='screen'
            
        ) ,
        
           launch_ros.actions.Node(
            package='rover_description',
            executable='wheel_steer_sub.py',
            output='screen'
            
        ),
           launch_ros.actions.Node(
            package='wheels_control',
            executable='NAV_displacement_cmds',
            output='screen'
            
        )

        ])



        
