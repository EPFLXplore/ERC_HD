#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from task_execution.task import PressButton, PlugVoltmeterAlign, PlugVoltmeterApproach, RassorSampling, BarMagnetApproach, EthernetApproach, AlignPanel, RockSamplingApproach, RockSamplingDrop, RockSamplingComplete
import task_execution.task
from task_execution.command import NamedJointTargetCommand
from hd_interfaces.msg import Task, Object, PoseGoal, JointSpaceCmd, TargetInstruction, MotorCommand
from avionics_interfaces.msg import ServoRequest, ServoResponse
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray, Int8, String, UInt32
import threading
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
import kinematics_utils.quaternion_arithmetic as qa
from typing import List, Type
from dataclasses import dataclass


@dataclass(frozen=True)
class TaskSelect:
    """simple wrapper for selecting a task"""
    info_msg: str
    task_type: Type[task_execution.task.Task]
    
    def select(self, executor):
        executor.loginfo(self.info_msg)
        return self.task_type(executor)


class Executor(Node):
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

    def __init__(self):
        super().__init__("kinematics_task_executor")
        self.createRosInterfaces()

        self.task: task_execution.task.Task = None
        self.new_task = False

        # indicates whether trajectory feedback has been updated
        self.traj_feedback_update = False
        # the feedback from the last MoveIt trajectory : True for success, False for fail (variable only usable when self.traj_feedback_update is True)
        self.traj_feedback = False

        self.received_voltmeter_response = False
    
    def createRosInterfaces(self):
        self.create_subscription(Task, "/HD/fsm/task_assignment", self.taskAssignementCallback, 10)
        self.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)
        self.create_subscription(TargetInstruction, "HD/vision/target_pose", pt.detected_object_pose_callback, 10)
        self.create_subscription(Bool, "/HD/kinematics/traj_feedback", self.trajFeedbackUpdate, 10)
        self.create_subscription(Int8, "/ROVER/Maintenance", self.CSMaintenanceCallback, 10)
        self.create_subscription(UInt32, "/HD/vision/depth", pt.depth_callback, 10)
        self.create_subscription(Float64MultiArray, "/HD/kinematics/set_camera_transform", pc.set_camera_transform_position, 10)
        self.create_subscription(ServoResponse, "/EL/servo_response", self.voltmeterResponseCallback, 10)
        self.pose_target_pub = self.create_publisher(PoseGoal, "/HD/kinematics/pose_goal", 10)
        self.joint_target_pub = self.create_publisher(Float64MultiArray, "/HD/kinematics/joint_goal", 10)
        self.add_object_pub = self.create_publisher(Object, "/HD/kinematics/add_object", 10)
        self.named_joint_target_pub = self.create_publisher(String, "/HD/kinematics/named_joint_target", 10)
        self.motor_command_pub = self.create_publisher(MotorCommand, "HD/kinematics/single_joint_cmd", 10)
        self.task_outcome_pub = self.create_publisher(Int8, "HD/kinematics/task_outcome", 10)
        self.voltmeter_pub = self.create_publisher(ServoRequest, "EL/servo_req", 10)
        self.joint_space_cmd_pub = self.create_publisher(JointSpaceCmd, "HD/kinematics/joint_goal2", 10)

    def loginfo(self, text: str):
        self.get_logger().info(text)
    
    def logwarn(self, text: str):
        self.get_logger().warn(text)
    
    def logerror(self, text: str):
        self.get_logger().error(text)
    
    def getTrajFeedback(self):
        """return the stored trajectory feedback"""
        self.traj_feedback_update = False
        return self.traj_feedback
    
    def waitForFeedback(self, hz: int=10) -> bool:
        """wait until trajectory feedback has been updated (it will be updated in a callback in another thread), then return the feedback"""
        rate = self.create_rate(hz)
        while not self.traj_feedback_update:
            rate.sleep()
        return self.getTrajFeedback()

    def waitForVoltmeterResponse(self, timeout: float=1.0, hz: int=10) -> bool:
        """
        wait until either timeout or a voltmeter response is received from avionics
        return: False if timed out else True
        """
        self.received_voltmeter_response = False
        rate = self.create_rate(hz)
        start = time.time()
        while not self.received_voltmeter_response:
            if time.time()-start > timeout:
                return False
            rate.sleep()
        return True

    def hasTask(self) -> bool:
        return self.task is not None

    def sendPoseGoal(self, goal: Pose, cartesian: bool=False, velocity_scaling_factor: float=1.0):
        """sends a pose goal to the trajectory planner"""
        msg = PoseGoal(
            goal = goal,
            cartesian = cartesian,
            velocity_scaling_factor = velocity_scaling_factor
        )
        self.pose_target_pub.publish(msg)
    
    def sendJointGoal(self, goal: List[float]):
        """sends a joint goal to the trajectory planner"""
        msg = Float64MultiArray(data=goal)
        self.joint_target_pub.publish(msg)
    
    def sendGripperTorque(self, torque_scaling_factor: float):
        """sends a gripper torque command to the motor control"""
        msg = MotorCommand(
            name = "Gripper",
            mode = MotorCommand.TORQUE,
            command = torque_scaling_factor
        )
        self.motor_command_pub.publish(msg)
    
    def sendRassorTorque(self, torque_scaling_factor: float):
        """sends a rassor torque command to the motor control"""
        msg = MotorCommand(
            name = "Rassor",
            mode = MotorCommand.TORQUE,
            command = torque_scaling_factor
        )
        self.motor_command_pub.publish(msg)

    def sendVoltmeterCommand(self, angle: float):
        """sends an extend/retract voltmeter command to Vincent's avionics"""
        msg = ServoRequest(destination_id=0, channel=1, angle=angle)
        self.voltmeter_pub.publish(msg)

    def sendJointSpaceCmd(self, mode: int, states: List[float]):
        """sends a joint space command which can be relative or absolute depending on the mode"""
        msg = JointSpaceCmd(
            mode = mode,
            states = Float64MultiArray(data=states)
        )
        self.joint_space_cmd_pub.publish(msg)

    def addObjectToWorld(self, shape: List[float], pose: Pose, name: str, type: int=Object.BOX, operation: int=Object.ADD):
        """sends an object to the planner, to be added to the world"""
        # TODO: modify this function and its interface to send a moveit_msgs.msg.CollisionObject instead of hd_interfaces.msg.Object + change its name to manipulate or something
        msg = Object(
            type = type,
            operation = operation,
            name = name,
            pose = pc.revert_from_vision(pose),
            shape = Float64MultiArray(data=shape)
        )
        self.add_object_pub.publish(msg)

    def sendNamedJointTarget(self, target: str):
        """named joint targets are predefined poses for the whole arm like "home" or "optimal_view" """
        msg = String(data=target)
        self.named_joint_target_pub.publish(msg)

    def trajFeedbackUpdate(self, msg: Bool):
        """listens to /HD/kinematics/traj_feedback topic"""
        self.traj_feedback_update = True
        self.traj_feedback = msg.data
    
    def voltmeterResponseCallback(self, msg: ServoResponse):
        """listens to /EL/servo_response topic"""
        if msg.success:
            self.received_voltmeter_response = True

    def taskAssignementCallback(self, msg: Task):
        """listens to /HD/fsm/task_assignment topic"""
        self.loginfo("Task executor received cmd")
        if self.hasTask():
            self.loginfo("but already has task : ignoring")
            return
        if msg.type not in self.KNOWN_TASKS:
            self.loginfo("Unknown task")
            return
        if msg.type == Task.NAMED_TARGET:
            self.task.addCommand(NamedJointTargetCommand(self, msg.str_slot))
        self.task = self.KNOWN_TASKS[msg.type].select(self)
        self.new_task = True
    
    def CSMaintenanceCallback(self, msg: Int8):
        """listens to /ROVER/Maintenance topic"""
        # TODO: put those constants in the message definition
        LAUNCH = 1
        ABORT = 2
        WAIT = 3
        RESUME = 4
        CANCEL = 5
        if msg.data == CANCEL or msg.data == ABORT:
            if self.hasTask():
                self.abortTask()

    def initiateTask(self):
        """starts assigned task"""
        self.new_task = False
        self.loginfo("Starting task")
        success = self.task.execute()
        msg = Int8()
        if success:
            msg.data = 0
            self.loginfo("Executed task successfully")
        else:
            msg.data = 1
            self.loginfo("Task failed")
        self.task_outcome_pub.publish(msg)
        self.task = None

    def abortTask(self):
        """stops the assigned task"""
        self.task.abort()
        self.task = None
    
    def testVision(self):
        if len(pt.DETECTED_OBJECTS_POSE) == 0: return
        shape = [0.2, 0.1, 0.0001]
        pose = pt.DETECTED_OBJECTS_POSE[0].artag_pose
        name = "test_btn"
        self.addObjectToWorld(shape, pose, name)

    def run(self):
        """main loop of the task executor"""
        rate = self.create_rate(25)   # 25hz
        while rclpy.ok():
            if self.new_task:
                # initiate the task in a new thread
                thread = threading.Thread(target=self.initiateTask)
                thread.start()
            self.testVision()
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
