#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from task_execution.all_tasks import PressButton
from task_execution.all_tasks import NamedJointTargetCommand
from kerby_interfaces.msg import Task, Object, PoseGoal
from hd_interfaces.msg import TargetInstruction
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray, Int8, String
import threading
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
import kinematics_utils.quaternion_arithmetic as qa


class Executor(Node):
    def __init__(self):
        super().__init__("kinematics_task_executor")
        self.create_subscription(Task, "/HD/fsm/task_assignment", self.taskAssignementCallback, 10)
        self.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)
        self.create_subscription(TargetInstruction, "target_pose", pt.detected_object_pose_callback, 10)
        self.create_subscription(Bool, "/HD/kinematics/traj_feedback", self.trajFeedbackUpdate, 10)
        self.pose_target_pub = self.create_publisher(PoseGoal, "/HD/kinematics/pose_goal", 10)
        self.joint_target_pub = self.create_publisher(Float64MultiArray, "/HD/kinematics/joint_goal", 10)
        self.add_object_pub = self.create_publisher(Object, "/HD/kinematics/add_object", 10)
        self.named_joint_target_pub = self.create_publisher(String, "/HD/kinematics/named_joint_target", 10)

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
        msg.pose = pc.revert_from_vision(pose)
        shape_ = Float64MultiArray()
        shape_.data = shape
        msg.shape = shape_
        self.add_object_pub.publish(msg)

    def sendNamedJointTarget(self, target: str):
        msg = String(data=target)
        self.named_joint_target_pub.publish(msg)

    def trajFeedbackUpdate(self, msg: Bool):
        self.traj_feedback_update = True
        self.traj_feedback = msg.data
    
    def manualInverseCallback(self, msg):
        if not self.hasTask():
            pass

    def taskAssignementCallback(self, msg: Task):
        """listens to /HD/fsm/task_assignment topic"""
        self.loginfo("Task executor received cmd")
        if self.hasTask():
            return
        if msg.description == "btn":
            self.loginfo("Button task")
            self.task = PressButton(self, msg.id, msg.pose, True)
        elif msg.description == "named_target":
            self.loginfo("Named target task")
            self.task = NamedJointTargetCommand(self, msg.str_slot)
        
        self.new_task = True

    def initiateTask(self):
        """starts assigned task"""
        self.new_task = False
        self.loginfo("Starting task")
        success = self.task.execute()
        if success:
            self.loginfo("Executed task successfully")
        else:
            self.loginfo("Task failed")
        self.task = None

    def abortTask(self):
        """stops the assigned task"""
        # TODO
    
    def testVision(self):
        if len(pt.DETECTED_OBJECTS_POSE) == 0: return
        shape = [0.2, 0.1, 0.0001]
        pose = pt.DETECTED_OBJECTS_POSE[0].artag_pose
        name = "test_btn"
        self.addObjectToWorld(shape, pose, name)

    def run(self):
        rate = self.create_rate(25)   # 25hz
        while rclpy.ok():
            if self.new_task:
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