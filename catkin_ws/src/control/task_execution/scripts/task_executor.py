#!/usr/bin/env python

import rospy
from task_execution.task_classes import *
from task_execution.msg import Task
import geometry_msgs.msg
import std_msgs.msg
import threading


class Executor:
    def __init__(self):
        self.task = None
        self.new_task = False

    def hasTask(self):
        if self.task is None:
            return False
        if isinstance(self.task, (PositionManualMotion, OrientationManualMotion)) and self.task.finished:
            return False
        return True

    def taskAssignementCallback(self, msg):
        """listens to /arm_control/task_assignment topic"""
        if self.hasTask():
            return
        if msg.description == "btn":
            self.task = PressButton(msg.pose)
            self.new_task = True
    
    def positionManualTaskCallback(self, msg):
        """listens to /arm_control/pos_manual_cmd topic"""
        if self.task is not None and not isinstance(self.task, PositionManualMotion):
            return
        if self.task is None or self.task.finished:
            print("NEW MANUAL COMMAND")
            self.task = PositionManualMotion()
            self.task.axis = (msg.data[:3])
            self.task.velocity_scaling = msg.data[3]
            self.new_task = True
            return
        print("PURSUE MANUAL COMMAND")
        self.task.pursue = True
        self.task.axis = (msg.data[:3])
        self.task.velocity_scaling = msg.data[3]
    
    def orientationManualTaskCallback(self, msg):
        """listens to /arm_control/orient_manual_cmd topic"""
        return  # TODO: remove return if feasible
        if self.task is not None and not isinstance(self.task, OrientationManualMotion):
            return
        if self.task is None or self.task.finished:
            print("NEW MANUAL COMMAND")
            self.task = OrientationManualMotion()
            self.task.axis = (msg.data[:3])
            self.task.velocity_scaling = msg.data[3]
            self.new_task = True
            return
        print("PURSUE MANUAL COMMAND")
        self.task.pursue = True
        self.task.axis = (msg.data[:3])
        self.task.velocity_scaling = msg.data[3]

    def assignTask(self, task):
        """assigns the task"""
        # TODO: implement or delete

    def initiateTask(self):
        """starts assigned task"""
        self.task.execute()
        self.task = None

    def abortTask(self):
        """stops the assigned task"""
        # TODO
    
    def run(self):
        rospy.Subscriber("/arm_control/task_assignment", Task, self.taskAssignementCallback)
        rospy.Subscriber("/arm_control/end_effector_pose", geometry_msgs.msg.Pose, register_end_effector_pose)
        rospy.Subscriber("/arm_control/pos_manual_inverse_cmd", std_msgs.msg.Float32MultiArray, self.positionManualTaskCallback)
        rospy.Subscriber("/arm_control/orient_manual_inverse_cmd", std_msgs.msg.Float32MultiArray, self.orientationManualTaskCallback)
        rate = rospy.Rate(25)   # 25hz
        while not rospy.is_shutdown():
            if self.new_task:
                self.new_task = False
                threading.Thread(target=self.initiateTask)
                self.initiateTask()
            rate.sleep()


if __name__ == "__main__":
    """init ros and subscribe to task commands from manager"""
    try:
        rospy.init_node('HD_control_task_executor', anonymous=True)
        exe = Executor()
        exe.run()
    except rospy.ROSInterruptException:
        rospy.logwarns("task executor crashed")
