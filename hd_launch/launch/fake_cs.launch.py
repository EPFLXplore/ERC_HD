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
import yaml


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



def generate_launch_description():
    fake_cs_arg, fake_cs_declaration = declare_binary_launch_argument("fake_cs", default=True, description="Run fake control station")
    keyboard_arg, keyboard_declaration = declare_binary_launch_argument("keyboard", default=False, description="Use keyboard as an input source for the fake CS (otherwise assuming gamepad)")
    
    hd_topic_names_file = get_package_file('custom_msg', 'config/hd_interface_names.yaml')
    rover_topic_names_file = get_package_file('custom_msg', 'config/rover_interface_names.yaml')
    
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

    # Declare all the steps of the launch file process
    return LaunchDescription([
        fake_cs_declaration,
        keyboard_declaration,
        fake_cs_node_gamepad,
        fake_cs_node_keyboard,
        ]
    )
