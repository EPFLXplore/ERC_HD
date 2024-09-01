import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    hd_topic_names_file = get_package_file('custom_msg', 'config/hd_interface_names.yaml')
    rover_topic_names_file = get_package_file('custom_msg', 'config/rover_interface_names.yaml')
    
    # planning_context
    
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("new_moveit_config"),
            "config",
            "kerby.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "new_moveit_config", "config/kerby.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "new_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "new_moveit_config", "config/moveit_controllers.yaml"
    )


    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    trajectory_execution.update(moveit_simple_controllers_yaml)

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    kerby_trajectory_planner_node = Node(
        package="trajectory_planner",
        executable="planner",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            trajectory_execution,
            planning_scene_monitor_parameters,
            robot_description_kinematics,
            hd_topic_names_file,
            rover_topic_names_file,
        ]
    )

    kerby_task_executor_node = Node(
        package="trajectory_planner",
        executable="task_executor.py",
        parameters=[
            hd_topic_names_file,
            rover_topic_names_file,
        ]
    )

    gripper_frame_broadcaster = Node(
        package='trajectory_planner',
        executable='gripper_frame_broadcaster.py',
    )
    
    return LaunchDescription(
        [
            kerby_trajectory_planner_node,
            kerby_task_executor_node,
            #gripper_frame_broadcaster,
        ]
    )
