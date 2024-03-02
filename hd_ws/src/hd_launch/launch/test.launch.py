from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def declare_binary_launch_argument(name, default=True, *, in_str_form=False):
    if not isinstance(default, bool):
        raise ValueError(f"Default value of binary launch argument should be of type bool not {type(default)}")
    arg = DeclareLaunchArgument(
        name, default_value=("true" if default else "false"),
        choices=["true", "false"]
    )
    return arg if in_str_form else (arg == "true")


def generate_launch_description():
    sim_arg = declare_binary_launch_argument("sim", default=False)
    rviz_arg = declare_binary_launch_argument("rviz", default=True, in_str_form=True)
    fake_cs_arg = declare_binary_launch_argument("fake_cs", default=True)

    kerby_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kerby_moveit_config'), 'launch'),
            '/kerby.launch.py']),
        launch_arguments={'rviz': rviz_arg}.items(),
    )

    trajectory_planner_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('trajectory_planner'), 'launch'),
            '/task_execution.launch.py']),
        launch_arguments={'rviz': rviz_arg}.items(),
    )

    fsm_node = Node(
        package="hd_fsm",
        executable="fsm"
    )

    fake_cs_node = Node(
        package="fake_components",
        executable="fake_cs_gamepad.py"
    )

    maybe_fake_cs = [fake_cs_node] if fake_cs_arg else []

    fake_motor_control_node = Node(
        package="fake_components",
        executable="fake_motor_control.py"
    )

    real_motor_control_node = Node(     # TODO: edit motor_control node in order to be runnable like that (without the path to the cofig)
        package="motor_control",
        executable="motor_control"
    )

    motor_control_node = fake_motor_control_node if sim_arg else real_motor_control_node


    return LaunchDescription([
        kerby_nodes,
        trajectory_planner_nodes,
        fsm_node,
        motor_control_node,
        ] + maybe_fake_cs
    )
