import os
 
from ament_index_python.packages import get_package_share_directory
 
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
 
 
 
def generate_launch_description():
 
    
    package_name='rover_description' 
    world_path1 = os.path.join(get_package_share_directory(package_name), "worlds", 'marsyard2022.world.xacro')
    world_path2 = os.path.join(get_package_share_directory(package_name), "worlds", 'simple.world')
    gazebo_launch=os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    mars_arg = DeclareLaunchArgument('mars', default_value='false', description='Launch Gazebo with Mars world')

                
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    ) 
 
   
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                   'world':world_path1,
                }.items()          
                 
    )


    gazebo1 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                 launch_arguments={
                    'world': world_path1  
                }.items(),
                condition=IfCondition(LaunchConfiguration('mars')),  
    )



    gazebo2 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                 launch_arguments={
                    #'world': world_path2  
                }.items(),
                condition=UnlessCondition(LaunchConfiguration('mars')),  
    )

    
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot', 
                                   '-x', '0.0',
                                    '-y', '0.0',
                                    '-z', '4.0',
                                    '-Y','0.0'
                                ],
                        output='screen'
                        )    

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )

    wheel_steering_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["position_controller"],
    )

    wheel_velocity_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller"],
    ) 
  
    hd_pos_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["hd_pos_controller"],
    ) 

    hd_vel_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["hd_vel_controller"],
    ) 
  


    return LaunchDescription([
        rsp,
        mars_arg,
        gazebo1,
        gazebo2,
        spawn_entity,
        joint_broad_spawner,
        # wheel_steering_spawner,
        # wheel_velocity_spawner,
        hd_pos_spawner,
        
    ])
