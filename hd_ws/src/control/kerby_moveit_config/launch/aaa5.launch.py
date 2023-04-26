import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
  moveit_config = (
    MoveItConfigsBuilder("kerby")
    .robot_description(file_path="config/kerby.urdf.xacro")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .to_moveit_configs()
  )

  run_move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict()],
  )

  # rviz_config_file = (
  #       get_package_share_directory("moveit2_tutorials") + "/launch/move_group.rviz"
  #   )
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    #arguments=["-d", rviz_config_file],
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
    ],
  )

  static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
  )

  # Publish TF
  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],
  )

  ros2_controllers_path = os.path.join(
    get_package_share_directory("kerby_moveit_config"),
    "config",
    "ros_controllers.yaml",
  )
  ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[moveit_config.robot_description, ros2_controllers_path],
    output="screen",
  )

  load_controllers = []
  for controller in [
    "rotate_joint_controller",
  ]:
    load_controllers += [
      ExecuteProcess(
        cmd=["ros2 run controller_manager spawner {}".format(controller)],
        shell=True,
        output="screen",
      )
    ]
  print(load_controllers);
  return LaunchDescription(
    [
      rviz_node,
      static_tf,
      robot_state_publisher,
      run_move_group_node,
      ros2_control_node
    ]
    + load_controllers
  )
