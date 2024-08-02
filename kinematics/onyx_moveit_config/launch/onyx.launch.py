import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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


def generate_launch_description():
    xacro_file = get_package_file('onyx_moveit_config', 'config/kerby.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('onyx_moveit_config', 'config/kerby.srdf')
    kinematics_file = get_package_file('onyx_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('onyx_moveit_config', 'config/ompl_planning.yaml')
    moveit_controllers_file = get_package_file('onyx_moveit_config', 'config/moveit_controllers.yaml')
    ros_controllers_file = get_package_file('onyx_moveit_config', 'config/ros2_controllers.yaml')
    joint_limits_file = get_package_file('onyx_moveit_config', 'config/joint_limits.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    servo_yaml = load_yaml2("onyx_moveit_config", "config/kerby_simulated_config.yaml")

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
    rviz_base = os.path.join(get_package_share_directory('onyx_moveit_config'), 'config')
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
    
    # Controller manager for realtime interactions
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters= [
            {'robot_description': robot_description},
            ros_controllers_file
        ],
        output="screen",
    )

    # Startup up ROS2 controllers (will exit immediately)
    controller_names = ['kerby_arm_moveit_controller', 'joint_state_broadcaster']   # kerby_base_moveit_controller
    spawn_controllers = [
        Node(
            package="controller_manager",
            executable="spawner",       # spawner.py on foxy
            arguments=[controller],
            output="screen")
        for controller in controller_names
    ]
    
    
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         {
            #             'moveit_servo': servo_yaml,
            #             'robot_description': robot_description,
            #             'robot_description_semantic': robot_description_semantic,
            #         }
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[{'robot_description': robot_description}],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::JoyToServoPub",
            #     name="controller_to_servo_node",
            # ),
        ],
        output="screen",
    )
    
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            {
                'moveit_servo': servo_yaml,
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_config,
            },
        ],
        output="screen",
    )
    

    return LaunchDescription([
        rviz_declaration,
        move_group_node,
        robot_state_publisher,
        ros2_control_node,
        rviz,
        # container,
        servo_node,
        ] + spawn_controllers
    )
