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


def generate_launch_description():
    truncated = True
    
    # planning_context
    if truncated:
        robot_description_config = xacro.process_file(
            os.path.join(
                get_package_share_directory("kerby_moveit_config"),
                "config_truncated",
                "kerby_truncated.urdf.xacro",
            )
        )
        robot_description = {"robot_description": robot_description_config.toxml()}

        robot_description_semantic_config = load_file(
            "kerby_moveit_config", "config_truncated/kerby_truncated.srdf"
        )
        robot_description_semantic = {
            "robot_description_semantic": robot_description_semantic_config
        }

        kinematics_yaml = load_yaml(
            "kerby_moveit_config", "config/kinematics.yaml"
        )
        robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

        # Trajectory Execution Functionality
        moveit_simple_controllers_yaml = load_yaml(
            "kerby_moveit_config", "config_truncated/moveit_controllers.yaml"
        )
    else:
        robot_description_config = xacro.process_file(
            os.path.join(
                get_package_share_directory("kerby_moveit_config"),
                "config",
                "kerby.urdf.xacro",
            )
        )
        robot_description = {"robot_description": robot_description_config.toxml()}

        robot_description_semantic_config = load_file(
            "kerby_moveit_config", "config/kerby.srdf"
        )
        robot_description_semantic = {
            "robot_description_semantic": robot_description_semantic_config
        }

        kinematics_yaml = load_yaml(
            "kerby_moveit_config", "config/kinematics.yaml"
        )
        robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

        # Trajectory Execution Functionality
        moveit_simple_controllers_yaml = load_yaml(
            "kerby_moveit_config", "config/moveit_controllers.yaml"
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
            robot_description_kinematics
        ]
    )

    kerby_task_executor_node = Node(
        package="trajectory_planner",
        executable="task_executor.py"
    )

    kerby_trajectory_planner_supervisor_node = Node(
        package="trajectory_planner",
        executable="planner_supervisor",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            trajectory_execution,
            planning_scene_monitor_parameters,
            robot_description_kinematics
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
            gripper_frame_broadcaster
            #kerby_trajectory_planner_supervisor_node,
        ]
    )