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
    rviz_arg, rviz_declaration = declare_binary_launch_argument("rviz", default=True, description="Run RViz")
    fake_cs_arg, fake_cs_declaration = declare_binary_launch_argument("fake_cs", default=True, description="Run fake control station")
    keyboard_arg, keyboard_declaration = declare_binary_launch_argument("keyboard", default=False, description="Use keyboard as an input source for the fake CS (otherwise assuming gamepad)")

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

    # Declare all the steps of the launch file process
    return LaunchDescription([
        sim_declaration,
        rviz_declaration,
        fake_cs_declaration,
        keyboard_declaration,
        kerby_nodes,
        trajectory_planner_nodes,
        fsm_node,
        fake_cs_node_gamepad,
        fake_cs_node_keyboard,
        fake_motor_control_node,
        real_motor_control_node
        ]
    )
