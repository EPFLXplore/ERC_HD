#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_planner.task_classes import PressButton
import trajectory_planner.pose_tracker as pt
from kerby_interfaces.msg import Task, Object, PoseGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray, Int8
import threading
from interfaces.msg import PanelObject
import trajectory_planner.eef_pose_corrector as epc
import trajectory_planner.quaternion_arithmetic as qa


class Executor(Node):
    def __init__(self):
        super().__init__("kinematics_task_executor")
        self.create_subscription(Task, "/HD/fsm/task_assignment", self.taskAssignementCallback, 10)
        self.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)
        self.create_subscription(PanelObject, "/HD/vision/distance_topic", pt.detected_object_pose_callback, 10)
        self.create_subscription(Bool, "/HD/kinematics/traj_feedback", self.trajFeedbackUpdate, 10)
        self.pose_target_pub = self.create_publisher(PoseGoal, "/HD/kinematics/pose_goal", 10)
        self.joint_target_pub = self.create_publisher(Float64MultiArray, "/HD/kinematics/joint_goal", 10)
        self.add_object_pub = self.create_publisher(Object, "/HD/kinematics/add_object", 10)

        self.task = None
        self.new_task = False

        self.traj_feedback_update = False
        self.traj_feedback = False

    def loginfo(self, text):
        self.get_logger().info(text)
    
    def logwarn(self, text):
        self.get_logger().warn(text)
    
    def logerror(self, text):
        self.get_logger().error(text)
    
    def getTrajFeedback(self):
        self.traj_feedback_update = False
        return self.traj_feedback
    
    def waitForFeedback(self, hz=10):
        rate = self.create_rate(hz)
        while not self.traj_feedback_update:
            rate.sleep()
        return self.getTrajFeedback()

    def hasTask(self):
        return self.task is not None

    """def add_panel(self, name, pose):
        dims = (2, 2, 0.05)
        msg = Object()
        msg.type == "box"
        msg.name = name
        msg.pose = pose
        msg.dims = dims
        self.add_obj_pub.publish(msg)"""

    def sendPoseGoal(self, goal: Pose, cartesian=False):
        msg = PoseGoal()
        msg.goal = goal
        msg.cartesian = cartesian
        self.pose_target_pub.publish(msg)
    
    def sendJointGoal(self, goal: list):
        msg = Float64MultiArray()
        msg.data = goal
        self.joint_target_pub.publish(msg)
    
    def addObjectToWorld(self, shape: list, pose: Pose, name: str, type="box"):
        msg = Object()
        msg.type = type
        msg.name = name
        msg.pose = epc.revert_from_vision(pose)
        shape_ = Float64MultiArray()
        shape_.data = shape
        msg.shape = shape_
        self.add_object_pub.publish(msg)

    def trajFeedbackUpdate(self, msg: Bool):
        self.traj_feedback_update = True
        self.traj_feedback = msg.data
        
    def taskAssignementCallback(self, msg: Task):
        """listens to /arm_control/task_assignment topic"""
        self.loginfo("Task executor received cmd")
        if self.hasTask():
            return
        if msg.description == "btn":
            self.loginfo("Button task")
            self.task = PressButton(self, msg.id, msg.pose, True)
            self.new_task = True

    def initiateTask(self):
        """starts assigned task"""
        self.loginfo("Starting task")
        self.task.execute()
        self.loginfo("Executed task")
        self.task = None

    def abortTask(self):
        """stops the assigned task"""
        # TODO
    
    def testVision(self):
        if len(pt.DETECTED_OBJECTS_POSE) == 0: return

        shape = [0.2, 0.1, 0.0001]
        relative_pose = pt.DETECTED_OBJECTS_POSE[0].object_pose
        pose = qa.compose_poses(epc.correct_eef_pose(), relative_pose)
        name = "test_btn"
        self.addObjectToWorld(shape, pose, name)

    def run(self):
        rate = self.create_rate(25)   # 25hz
        while rclpy.ok():
            if self.new_task:
                self.new_task = False
                thread = threading.Thread(target=self.initiateTask)
                thread.start()
            #self.testVision()
            rate.sleep()


def main():
    rclpy.init()
    executor = Executor()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(executor, ), daemon=True)
    thread.start()

    try:
        executor.run()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
