import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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
    print("enter")
    robot_description_config = load_file(
        "astra_description", "urdf/astra_generated.urdf"
    )
    robot_description = {"robot_description": robot_description_config}
    print("ROBOT DESCRIPTION")

    robot_description_semantic_config = load_file(
        "astra_moveit_config", "config/astra.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }
    print("ROBOT DESCRIPTION SEMANTIC")

    kinematics_yaml = load_yaml(
        "astra_moveit_config", "config/kinematics.yaml"
    )
    print("KINEMATICS")

    planning_yaml = load_yaml(
        "astra_moveit_config", "config/ompl_planning.yaml"
    )
    print("PLANNING")

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}
    print("PLANNNING PLUGIN")

    res = LaunchDescription(
        [
            Node(
                package="moveit_api_test",
                executable="motion_planning_test",
                name="motion_planning_test",
                parameters=[
                    #robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_yaml,
                    planning_plugin,
                ],
            )
        ]
    )
    print("RES")
    return res