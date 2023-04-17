import sys
import copy
import time
import math
import rclpy
from rclpy.node import Node
import threading
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import astra_interfaces.msg
from astra_interfaces.srv import PoseGoal, PoseGoalResponse, JointGoal, JointGoalResponse


class Planner(Node):
    """recieves goals to reach and sends information to MoveIt in order for it to compute a trajectory"""
    NONE = 0
    JOINT_GOAL = 1
    POSE_GOAL = 2
    CARTESIAN_GOAL = 3

    def __init__(self):
        # initialize ROS ===============================================================================================
        moveit_commander.roscpp_initialize(sys.argv)

        self.create_ros_interfaces()
    
    def create_ros_interfaces(self):
        # publishers ===================================================================================================
        self.end_of_mvt_pub = self.create_publisher(astra_interfaces.msg.cmdOutcome, "/arm_control/end_of_movement", queue_size=5)  # TODO: probably remove this
        self.end_effector_pose_pub = self.create_publisher(geometry_msgs.msg.Pose, "/arm_control/end_effector_pose", queue_size=5)
        self.display_trajectory_pub = self.create_publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # subscribers ==================================================================================================
        self.create_subscription(PoseGoal, "/arm_control/pose_goal", self.handle_pose_goal)
        self.create_subscription(JointGoal, "/arm_control/joint_goal", self.handle_joint_goal)
        #rospy.Subscriber("/arm_control/pose_goal", PoseGoal, self.handle_pose_goal)
        #rospy.Subscriber("/arm_control/joint_goal", JointGoal, self.handle_joint_goal)
        self.create_subscription(astra_interfaces.msg.Object, "/arm_control/add_object", self.add_object_callback)
        self.create_subscription(std_msgs.msg.String, "/arm_control/remove_object", self.remove_object_callback)
        self.create_subscription(sensor_msgs.msg.JointState, "/arm_control/joint_telemetry", self.telemetry_callback)
        self.create_subscription(std_msgs.msg.Bool, "/arm_control/show_lidar", self.show_lidar_callback)

    def run(self):
        """main"""
        spin = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin.start()
        rate = self.create_rate(2)   # 25hz
        print("ZEBI")

        """if VERBOSE:
            rospy.logwarn("manager started")"""
        try:
            while rclpy.ok():
                if self.mode_transitioning:
                    self.transition_loop_action()
                else:
                    self.normal_loop_action()
                rate.sleep()
        except KeyboardInterrupt:
            pass
        
        self.destroy_node()
        rclpy.shutdown()
        spin.join()


def main(args=None):
    rclpy.init(args)
    Planner().run()

    
if __name__ == "__main__":
    rclpy.init()
