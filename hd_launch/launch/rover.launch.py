from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from typing import List, Tuple


def declare_launch_argument(name: str, default_value: str, choices: List[str] = None, description: str = "") -> Tuple[LaunchConfiguration, DeclareLaunchArgument]:
    # Initialize the LaunchConfiguration
    arg = LaunchConfiguration(name)

    # Declare the launch argument with a default value
    declare_arg = DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description,
        choices=choices
    )
    
    return arg, declare_arg


def declare_binary_launch_argument(name: str, default: bool = True, description: str="", allow_none: bool = True):
    if not isinstance(default, bool):
        raise ValueError(f"Default value of binary launch argument should be of type bool not {type(default)}")
    
    arg, declare_arg = declare_launch_argument(
        name,
        default_value='True' if default else 'False',
        description=description,
        choices=["True", "False", "None"] if allow_none else ["True", "False"]
    )

    return arg, declare_arg


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path



def generate_launch_description():
    sim_arg, sim_declaration = declare_binary_launch_argument("sim", default=False, description="Run in simulation mode")
    rviz_arg, rviz_declaration = declare_binary_launch_argument("rviz", default=False, description="Run RViz")

    hd_topic_names_file = get_package_file('custom_msg', 'config/hd_interface_names.yaml')
    rover_topic_names_file = get_package_file('custom_msg', 'config/rover_interface_names.yaml')
    
    kerby_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('new_moveit_config'), 'launch'),
            '/onyx.launch.py']),
        launch_arguments={'rviz': rviz_arg}.items(),
    )

    trajectory_planner_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('trajectory_planner'), 'launch'),
            '/new_task_execution.launch.py']),
    )

    fsm_node = Node(
        package="hd_fsm",
        executable="fsm",
        parameters=[
            hd_topic_names_file,
            rover_topic_names_file,
        ]
    )

    fake_motor_control_node = Node(
        package="fake_components",
        executable="fake_motor_control.py",
        condition=IfCondition(PythonExpression([sim_arg, "== True"])) # Run if we are in simulation
    )

    real_motor_control_node = Node(     # TODO: edit motor_control node in order to be runnable like that (without the path to the cofig)
        package="ethercat_device_configurator",
        executable="motor_control",
        condition=IfCondition(PythonExpression([sim_arg, "== False"])) # Do not run if we are in simulation
    )
    
    # Declare the model path as a launch argument
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',  # Set a default path or make it mandatory by removing `default_value`
        description='Path to the YOLO model file'
    )

    # Log the model path to ensure it's being passed correctly
    # log_model_path = LogInfo(msg=LaunchConfiguration('model_path'))

    model_node = Node(
        package='perception',  # perception
        executable='model_node',  
        name='model_node',
        output='screen',
        parameters=[hd_topic_names_file]  # {'model_path': LaunchConfiguration('model_path')},
    )

    perception_node = Node(package="perception", executable="perception_node", name="perception", parameters=[hd_topic_names_file])
    
    camera_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('camera'),
                'launch/camera_node.launch.py'))
    )
    
    # Declare all the steps of the launch file process
    return LaunchDescription([
        model_path_arg,
        sim_declaration,
        rviz_declaration,
        kerby_nodes,
        trajectory_planner_nodes,
        fsm_node,
        fake_motor_control_node,
        real_motor_control_node,
        model_node,
        perception_node,
        camera_nodes,
        ]
    )
