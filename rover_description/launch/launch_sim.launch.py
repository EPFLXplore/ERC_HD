import os
import yaml

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
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    mars_arg = DeclareLaunchArgument('mars', default_value='false', description='Launch Gazebo with Mars world')


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
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
                    'world': world_path2
                }.items(),
                condition=UnlessCondition(LaunchConfiguration('mars')),
    )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'onyx',
                                   '-x', '0.0',
                                    '-y', '0.0',
                                    '-z', '4.0',
                                    '-Y','0.0'
                                ],
                        output='screen'
                        )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kerby_arm_moveit_controller"],
    )




    return LaunchDescription(
        [
            rsp,
            mars_arg,
            gazebo1,
            gazebo2,
            spawn_entity,
            joint_broad_spawner,
            arm_controller,
        ]
        + more_launch_description()
        + more_more_launch_description()
    )






def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml2(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def declare_binary_launch_argument(name, default=True, description=""):
    if not isinstance(default, bool):
        raise ValueError(f"Default value of binary launch argument should be of type bool not {type(default)}")
    
    # Initialize the LaunchConfiguration
    arg = LaunchConfiguration(name)

    # Declare the launch argument with a default value
    declare_arg = DeclareLaunchArgument(
        name,
        default_value='True' if default else 'False',
        description=description,
        choices=['True', 'False']
    )
    return arg, declare_arg
    
    

def more_launch_description():
    xacro_file = get_package_file('rover_description', 'description/onyx.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('new_moveit_config', 'config/kerby.srdf')
    kinematics_file = get_package_file('new_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('new_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file('new_moveit_config', 'config/moveit_controllers.yaml')
    ros_controllers_file = get_package_file('new_moveit_config', 'config/ros2_controllers.yaml')
    joint_limits_file = get_package_file('new_moveit_config', 'config/joint_limits.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    servo_yaml = load_yaml2("new_moveit_config", "config/kerby_simulated_config.yaml")

    rviz_arg, rviz_declaration = declare_binary_launch_argument("rviz", default=True, description="Run RViz")

    joint_limits = {"robot_description_planning": load_yaml(joint_limits_file)}

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    trajectory_execution.update(load_yaml(moveit_controllers_file))

    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }
    
    
    # MoveIt node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
                'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
                "use_sim_time": True,
            },
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
            joint_limits,
        ],
    )
    
    # TF information
    robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    
    # Visualization (parameters needed for MoveIt display plugin)
    rviz_base = os.path.join(get_package_share_directory('new_moveit_config'), 'config')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_full_config],
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            },
            joint_limits,
        ],
        condition=IfCondition(PythonExpression([rviz_arg, "== True"]))
    )
    
    
    return [
        rviz_declaration,
        move_group_node,
        # robot_state_publisher,
        rviz,
        # servo_node,
    ]



def more_more_launch_description():
    sim_arg, sim_declaration = declare_binary_launch_argument("sim", default=False, description="Run in simulation mode")
    fake_cs_arg, fake_cs_declaration = declare_binary_launch_argument("fake_cs", default=True, description="Run fake control station")
    keyboard_arg, keyboard_declaration = declare_binary_launch_argument("keyboard", default=False, description="Use keyboard as an input source for the fake CS (otherwise assuming gamepad)")

    hd_topic_names_file = get_package_file('custom_msg', 'config/hd_interface_names.yaml')
    rover_topic_names_file = get_package_file('custom_msg', 'config/rover_interface_names.yaml')
    
    
    fsm_node = Node(
        package="hd_fsm",
        executable="fsm",
        parameters=[
            hd_topic_names_file,
            rover_topic_names_file,
        ]
    )

    fake_cs_node_gamepad = Node(
        package="fake_components",
        executable="new_fake_cs.py",    # "fake_cs_gamepad.py",
        condition=IfCondition(PythonExpression([fake_cs_arg, "== True and ", keyboard_arg, "== False"])), # Run if the fake CS is needed
        parameters=[
            {"input_device": "gamepad"},
            hd_topic_names_file,
            rover_topic_names_file,
        ]
    )

    fake_cs_node_keyboard = Node(
        package="fake_components",
        executable="new_fake_cs.py",    # "fake_cs_gamepad.py",
        condition=IfCondition(PythonExpression([fake_cs_arg, "== True and ", keyboard_arg, "== True"])), # Run if the fake CS is needed
        parameters=[
            {"input_device": "keyboard"},
            hd_topic_names_file,
            rover_topic_names_file,
        ]
    )
    
    return [
        sim_declaration,
        fake_cs_declaration,
        keyboard_declaration,
        # trajectory_planner_nodes,
        fsm_node,
        fake_cs_node_gamepad,
        fake_cs_node_keyboard,
        # fake_motor_control_node,
        # real_motor_control_node,
    ]