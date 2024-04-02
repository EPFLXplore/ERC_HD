import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument


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

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def declare_binary_launch_argument(name, default=True, *, in_str_form=False):
    if not isinstance(default, bool):
        raise ValueError(f"Default value of binary launch argument should be of type bool not {type(default)}")
    arg = DeclareLaunchArgument(
        name, default_value=("true" if default else "false"),
        choices=["true", "false"]
    )
    return arg if in_str_form else (arg == "true")


def generate_launch_description():
    xacro_file = get_package_file('sloan_description', 'urdf/sloan.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    #srdf_file = get_package_file('kerby_moveit_config', 'config/kerby.srdf')
    #kinematics_file = get_package_file('kerby_moveit_config', 'config/kinematics.yaml')
    #joint_limits_file = get_package_file('kerby_moveit_config', 'config/joint_limits.yaml')

    robot_description = load_file(urdf_file)
    #robot_description_semantic = load_file(srdf_file)
    #kinematics_config = load_yaml(kinematics_file)

    #joint_limits = {"robot_description_planning": load_yaml(joint_limits_file)}

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
    rviz_base = os.path.join(get_package_share_directory('kerby_moveit_config'), 'config')
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
                #'robot_description_semantic': robot_description_semantic,
                #'robot_description_kinematics': kinematics_config,
            },
            #joint_limits,
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz
        ]
    )
