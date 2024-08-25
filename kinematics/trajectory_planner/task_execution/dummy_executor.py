#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from task_execution.task import PressButton, PlugVoltmeterAlign, PlugVoltmeterApproach, RassorSampling, BarMagnetApproach, EthernetApproach, AlignPanel, RockSamplingApproach, RockSamplingDrop, RockSamplingComplete
import task_execution.task
from task_execution.command import NamedJointTargetCommand
from custom_msg.msg import Task, Object, PoseGoal, JointSpaceCmd, TargetInstruction, MotorCommand
from custom_msg.msg import ServoRequest, ServoResponse
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray, Int8, String, UInt32
import threading
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as new_qa
from typing import List, Type
from dataclasses import dataclass


# class Object:
#     # object types
#     BOX = 0

#     # operations
#     ADD = 0
#     REMOVE = 1


class Rate:
    def __init__(self, hz):
        self.hz = hz
        self.period = 1/self.hz
        self.last_query_time = time.time()
    
    def sleep(self):
        t = time.time()
        elapsed = t - self.last_query_time
        time.sleep(max(0.0, self.period - elapsed))
        self.last_query_time = t


@dataclass(frozen=True)
class TaskSelect:
    """simple wrapper for selecting a task"""
    info_msg: str
    task_type: Type[task_execution.task.Task]
    
    def select(self, executor):
        executor.loginfo(self.info_msg)
        return self.task_type(executor)


class DummyExecutor:
    KNOWN_TASKS = {
        Task.BUTTON:                    TaskSelect("Button task",               PressButton),
        Task.PLUG_VOLTMETER_ALIGN:      TaskSelect("Plug voltmeter task",       PlugVoltmeterAlign),
        Task.METAL_BAR_APPROACH:        TaskSelect("Metal bar approach task",   BarMagnetApproach),
        Task.NAMED_TARGET:              TaskSelect("Named target task",         task_execution.task.Task),
        Task.RASSOR_SAMPLE:             TaskSelect("Rassor sample task",        RassorSampling),
        Task.ETHERNET_CABLE:            TaskSelect("Plug ethernet task",        EthernetApproach),
        Task.ALIGN_PANEL:               TaskSelect("Align panel",               AlignPanel),
        Task.ROCK_SAMPLING_APPROACH:    TaskSelect("Rock sampling approach",    RockSamplingApproach),
        Task.ROCK_SAMPLING_DROP:        TaskSelect("Rock sampling drop",        RockSamplingDrop),
        Task.ROCK_SAMPLING_COMPLETE:    TaskSelect("Complete rock sampling",    RockSamplingComplete),
    }

    NAMED_POSES = {
        "home": new_qa.Point(x=0.0, y=0.0, z=0.0),
    }

    def __init__(self):
        self.task: task_execution.task.Task = None
        self.new_task = False

        self.current_pos = new_qa.Point()
        self.goal_pos = new_qa.Point()

    def loginfo(self, text: str):
        print("[INFO]:", text)

    def logwarn(self, text: str):
        print("[WARN]:", text)
    
    def logerror(self, text: str):
        print("[ERROR]:", text)
    
    def create_rate(self, hz: int):
        return Rate(hz)
    
    def getTrajFeedback(self):
        """return the stored trajectory feedback"""
        self.traj_feedback_update = False
        return self.traj_feedback
    
    def waitForFeedback(self, hz: int=10) -> bool:
        """wait until trajectory feedback has been updated (it will be updated in a callback in another thread), then return the feedback"""
        duration = 3
        n_steps = 100
        for i in range(n_steps+1):
            current_pos = self.current_pos + i/n_steps * (self.goal_pos-self.current_pos)
            if i > 0: print("\r", end="")
            print(f"Arm is moving... Current position: {current_pos}", end="", flush=True)
            time.sleep(duration/n_steps)
        print()
        return True

    def waitForVoltmeterResponse(self, timeout: float=1.0, hz: int=10) -> bool:
        """
        wait until either timeout or a voltmeter response is received from avionics
        return: False if timed out else True
        """
        duration = 3
        n_steps = 100
        for i in range(n_steps+1):
            if i > 0: print("\r", end="")
            print(f"Waiting for voltmeter response {'.'*(i%4)}", end="", flush=True)
            time.sleep(duration/n_steps)
        print()
        return True

    def hasTask(self) -> bool:
        return self.task is not None

    def sendPoseGoal(self, goal: Pose, cartesian: bool=False, velocity_scaling_factor: float=1.0):
        """sends a pose goal to the trajectory planner"""
        self.goal_pos = new_qa.Point.make(goal.position)
    
    def sendJointGoal(self, goal: List[float]):
        """sends a joint goal to the trajectory planner"""
        self.goal_pos = new_qa.Point.random()
    
    def sendGripperTorque(self, torque_scaling_factor: float):
        """sends a gripper torque command to the motor control"""
        print(f"Sending gripper torque {torque_scaling_factor}")
    
    def sendRassorTorque(self, torque_scaling_factor: float):
        """sends a rassor torque command to the motor control"""
        print(f"Sending rassor torque {torque_scaling_factor}")

    def sendVoltmeterCommand(self, angle: float):
        """sends an extend/retract voltmeter command to Vincent's avionics"""
        print(f"Sending voltmeter angle {angle}")

    def sendJointSpaceCmd(self, mode: int, states: List[float]):
        """sends a joint space command which can be relative or absolute depending on the mode"""
        self.goal_pos = new_qa.Point.random()

    def addObjectToWorld(self, shape: List[float], pose: Pose, name: str, type: int=Object.BOX, operation: int=Object.ADD):
        """sends an object to the planner, to be added to the world"""
        # TODO: modify this function and its interface to send a moveit_msgs.msg.CollisionObject instead of custom_msg.msg.Object + change its name to manipulate or something
        print("Adding an object to the world")
    
    def detectionUpdated(self):
        pt.DETECTED_OBJECTS_POSE.clear()
        obj = pt.DetectedObject()
        obj.artag_pose = new_qa.Pose.random()
        obj.object_pose = new_qa.Pose.random()
        pt.DETECTED_OBJECTS_POSE.append(obj)
        pt.DETECTION_UPDATED = True
        return True
    
    def sendNamedJointTarget(self, target: str):
        """named joint targets are predefined poses for the whole arm like "home" or "optimal_view" """
        print(f"Sending target {target}")
        self.goal_pos = self.NAMED_POSES[target]

    def taskAssignementCallback(self, task_type: int, str_slot: str=""):
        self.loginfo("Task executor received cmd")
        if self.hasTask():
            self.loginfo("but already has task : ignoring")
            return
        if task_type not in self.KNOWN_TASKS:
            self.loginfo("Unknown task")
            return
        self.task = self.KNOWN_TASKS[task_type].select(self)
        if task_type == Task.NAMED_TARGET:
            self.task.addCommand(NamedJointTargetCommand(self, str_slot))
        self.new_task = True

    def initiateTask(self):
        """starts assigned task"""
        self.new_task = False
        self.loginfo("Starting task")
        success = self.task.execute()
        msg = Int8()
        if success:
            msg.data = 0
            self.loginfo("Executed task successfully\n")
        else:
            msg.data = 1
            self.loginfo("Task failed")
        #self.task_outcome_pub.publish(msg)
        self.task = None

    def abortTask(self):
        """stops the assigned task"""
        self.task.abort()
        self.task = None

    def run(self):
        """main loop of the task executor"""
        rate = Rate(25)     # 25hz
        while True:
            if self.new_task:
                # initiate the task in a new thread
                thread = threading.Thread(target=self.initiateTask)
                thread.start()
            #self.testVision()
            rate.sleep()
    
    def selector(self):
        while True:
            l = input("Input task:").split()
            l[0] = int(l[0])
            self.taskAssignementCallback(*l)
    
    def emulate(self):
        thread = threading.Thread(target=self.selector, daemon=True)
        thread.start()
        self.run()


def main():
    executor = DummyExecutor()
    executor.emulate()


if __name__ == "__main__":
    main()
