from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    arg_show_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="start RViz automatically with the launch file",
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kerby_moveit_config"), "config", "kerby.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content.toxml()}

    kerby_moveit_controller = PathJoinSubstitution(
        [
            FindPackageShare("kerby_moveit_config"),
            "config",
            "moveit_controllers.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, kerby_moveit_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["kerby_arm_moveit_controller"],
        output="screen",
    )
    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kerby_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            arg_show_rviz,
            node_robot_state_publisher,
            # controller_manager_node,
            # spawn_dd_controller,
            # spawn_jsb_controller,
            rviz_node,
        ]
    )