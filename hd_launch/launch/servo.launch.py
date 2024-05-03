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
    sim_arg, sim_declaration = declare_binary_launch_argument("sim", default=False, description="Run in simulation mode")
    rviz_arg, rviz_declaration = declare_binary_launch_argument("rviz", default=True, description="Run RViz")
    fake_cs_arg, fake_cs_declaration = declare_binary_launch_argument("fake_cs", default=True, description="Run fake control station")

    kerby_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kerby_moveit_config'), 'launch'),
            '/servo.launch.py']),
        launch_arguments={'rviz': rviz_arg}.items(),
    )

    # trajectory_planner_nodes = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('trajectory_planner'), 'launch'),
    #         '/task_execution.launch.py']),
    # )

    fsm_node = Node(
        package="hd_fsm",
        executable="fsm"
    )

    fake_cs_node = Node(
        package="fake_components",
        executable="servo_fake_cs_gamepad.py",
        condition=IfCondition(PythonExpression([fake_cs_arg, "== True"])) # Run if the fake CS is needed 
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
        kerby_nodes,
        #trajectory_planner_nodes,
        fsm_node,
        fake_cs_node,
        fake_motor_control_node,
        real_motor_control_node
        ]
    )
