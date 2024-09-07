#!/usr/bin/env python3

from __future__ import annotations
import rclpy
import time
from rclpy.node import Node
# from task_execution.task import PressButton, PlugVoltmeterAlign, PlugVoltmeterApproach, RassorSampling, BarMagnetApproach, EthernetApproach, AlignPanel, RockSamplingApproach, RockSamplingDrop, RockSamplingComplete, Dummy
import task_execution.task as task
# from task_execution.command import NamedJointTargetCommand
import task_execution.command as command
from custom_msg.msg import (Task, Object, PoseGoal, JointSpaceCmd, ArucoObject, 
                            MotorCommand, HDGoal, ServoRequest, ServoResponse, Rock, RockArray, Brick, Ethernet, Probe)
from custom_msg.srv import RequestHDGoal
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray, Int8, String, UInt32
from std_srvs.srv import Trigger
import threading
import kinematics_utils.pose_tracker as pt
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
from typing import List, Type
from dataclasses import dataclass
from rclpy.task import Future


@dataclass(frozen=True)
class TaskSelect:
    """simple wrapper for selecting a task"""
    info_msg: str
    task_type: Type[task.Task]
    
    def select(self, executor: Executor, **kwargs):
        executor.loginfo(self.info_msg)
        return self.task_type(executor, **kwargs)


class DoneFlag:
    def __init__(self):
        self.done = False
        self.future: Future = None
    
    def trigger(self, future: Future):
        self.done = True
        self.future = future
    
    def __bool__(self) -> bool:
        return self.done


class Executor(Node):
    INSTANCE: Executor = None
    KNOWN_TASKS = {
        Task.BUTTON:                    TaskSelect("Button task",                   task.PressButton),#DummyWithTools),
        Task.PLUG_VOLTMETER_ALIGN:      TaskSelect("Plug voltmeter task",           task.PlugVoltmeterAlign),
        Task.METAL_BAR_APPROACH:        TaskSelect("Metal bar approach task",       task.BarMagnetApproach),
        Task.NAMED_TARGET:              TaskSelect("Named target task",             task.Task),
        Task.RASSOR_SAMPLE:             TaskSelect("Rassor sample task",            task.RassorSampling),
        Task.ETHERNET_CABLE:            TaskSelect("Plug ethernet task",            task.EthernetApproach),
        Task.ALIGN_PANEL:               TaskSelect("Align panel",                   task.AlignPanel),
        # Task.ROCK_SAMPLING_APPROACH:    TaskSelect("Rock sampling approach",        task.RockSamplingApproach),
        # Task.ROCK_SAMPLING_DROP:        TaskSelect("Rock sampling drop",            task.RockSamplingDrop),
        # Task.ROCK_SAMPLING_COMPLETE:    TaskSelect("Complete rock sampling",        task.RockSamplingComplete),
    }
    KNOWN_TASKS_NEW = {
        btn_task:                       TaskSelect("Button task",                   task.PressButton)
        for btn_task in [HDGoal.BUTTON_A0, HDGoal.BUTTON_A1, HDGoal.BUTTON_A2, HDGoal.BUTTON_A3, HDGoal.BUTTON_A4, HDGoal.BUTTON_A5, HDGoal.BUTTON_A6, HDGoal.BUTTON_A7, HDGoal.BUTTON_A8, HDGoal.BUTTON_A9, HDGoal.BUTTON_B1]
    } | {
        HDGoal.TOOL_PICKUP:             TaskSelect("Pick tool up task",             task.ToolPickup),
        HDGoal.TOOL_PLACEBACK:          TaskSelect("Place tool back task",          task.ToolPlaceback),
        HDGoal.PREDEFINED_POSE:         TaskSelect("Predefined target pose task",   task.PredefinedTargetPose),
        HDGoal.DROP_SAMPLE:             TaskSelect("Drop sample task",              task.DropSample),
        HDGoal.ROCK:                    TaskSelect("Approach rock",                 task.RockPicking),
        HDGoal.VOLTMETER_ALIGN:         TaskSelect("Voltmeter align",               task.VoltmeterAlignNew),
    } | {
        
    }

    def __new__(cls):
        if cls.INSTANCE is not None:
            return cls.INSTANCE
            raise RuntimeError("Executor class can only have one instance")
        instance = super().__new__(cls)
        cls.INSTANCE = instance
        return instance
    
    @classmethod
    def get_instance(cls) -> Executor:
        return cls.INSTANCE
    
    def __init__(self):
        super().__init__("kinematics_task_executor")
        self.createRosInterfaces()

        self.task: task.Task = None
        self.new_task = False

        # indicates whether trajectory feedback has been updated
        self.traj_feedback_update = False
        # the feedback from the last MoveIt trajectory : True for success, False for fail (variable only usable when self.traj_feedback_update is True)
        self.traj_feedback = False

        self.received_voltmeter_response = False
    
    def get_str_param(self, name: str, default: str = "") -> str:
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().string_value
    
    def createRosInterfaces(self):
        self.create_subscription(Task, "/HD/fsm/task_assignment", self.taskAssignmentCallback, 10)
        self.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)
        self.create_subscription(ArucoObject, "/HD/perception/button_pose", pt.detected_object_pose_callback, 10)
        self.create_subscription(Bool, "/HD/kinematics/traj_feedback", self.trajFeedbackUpdate, 10)
        self.create_subscription(Int8, "/ROVER/Maintenance", self.CSMaintenanceCallback, 10)
        self.create_subscription(UInt32, "/HD/vision/depth", pt.depth_callback, 10)
        self.create_subscription(Float64MultiArray, "/HD/kinematics/set_camera_transform", pc.set_camera_transform_position, 10)
        self.create_subscription(ServoResponse, "/EL/servo_response", self.voltmeterResponseCallback, 10)
        self.create_subscription(RockArray, self.get_str_param("hd_perception_rocks"), pt.perception_tracker.rock_detection.callback, 10)
        # self.create_subscription(ArucoObject, self.get_str_param("hd_perception_rocks"), pt.perception_tracker.aruco_object_detection.callback, 10)
        self.create_subscription(Int8, self.get_str_param("hd_fsm_abort_topic"), self.abortCallback, 10)
        
        self.pose_target_pub = self.create_publisher(PoseGoal, "/HD/kinematics/pose_goal", 10)
        self.joint_target_pub = self.create_publisher(Float64MultiArray, "/HD/kinematics/joint_goal", 10)
        self.add_object_pub = self.create_publisher(Object, "/HD/kinematics/add_object", 10)
        self.attach_object_pub = self.create_publisher(Object, "/HD/kinematics/attach_object", 10)
        self.named_joint_target_pub = self.create_publisher(String, "/HD/kinematics/named_joint_target", 10)
        self.motor_command_pub = self.create_publisher(MotorCommand, "HD/kinematics/single_joint_cmd", 10)
        self.task_outcome_pub = self.create_publisher(Int8, "HD/kinematics/task_outcome", 10)
        self.voltmeter_pub = self.create_publisher(ServoRequest, "EL/servo_req", 10)
        self.joint_space_cmd_pub = self.create_publisher(JointSpaceCmd, "HD/kinematics/joint_goal2", 10)
        
        self.goal_assignment_srv = self.create_service(RequestHDGoal, self.get_str_param("hd_task_executor_goal_srv"), self.goalAssignmentCallback)

        self.human_verification_cli = self.create_client(Trigger, self.get_str_param("rover_hd_human_verification_srv"))
        
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
            goal = qan.publishable(goal),
            cartesian = cartesian,
            velocity_scaling_factor = velocity_scaling_factor
        )
        self.pose_target_pub.publish(msg)
    
    def sendJointGoal(self, goal: List[float]):
        """sends a joint goal to the trajectory planner"""
        # TODO: this method may be an outdated equivalent of sendJointSpaceCmd
        msg = Float64MultiArray(data=goal)
        self.joint_target_pub.publish(msg)
    
    def sendGripperTorque(self, torque_scaling_factor: float):
        """sends a gripper torque command to the motor control"""
        msg = MotorCommand(
            name = "Gripper",
            mode = MotorCommand.TORQUE,
            command = float(torque_scaling_factor)
        )
        self.motor_command_pub.publish(msg)
    
    def sendRassorTorque(self, torque_scaling_factor: float):
        """sends a rassor torque command to the motor control"""
        msg = MotorCommand(
            name = "Rassor",
            mode = MotorCommand.TORQUE,
            command = float(torque_scaling_factor)
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
    
    def requestHumanVerification(self) -> bool:
        req = Trigger.Request()
        while not self.human_verification_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        future = self.human_verification_cli.call_async(req)
        done_flag = DoneFlag(self)
        future.add_done_callback(done_flag.trigger)
        
        while not done_flag:
            time.sleep(0.2)
        
        result: Trigger.Response = done_flag.future.result()
        return result.success

    def addObjectToWorld(self, shape: List[float], pose: Pose, name: str, type: int=Object.BOX, operation: int=Object.ADD):
        """sends an object to the planner, to be added to the world"""
        # TODO: modify this function and its interface to send a moveit_msgs.msg.CollisionObject instead of custom_msg.msg.Object + change its name to manipulate or something
        msg = Object(
            type = type,
            operation = operation,
            name = name,
            pose = pc.revert_from_vision(pose).publishable(),
            shape = Float64MultiArray(data=shape)
        )
        self.add_object_pub.publish(msg)
    
    def attachObjectToGripper(self, shape: List[float], pose: Pose, name: str, type: int=Object.BOX, operation: int=Object.ADD):
        """sends an object to the planner, to be added to the world"""
        # TODO: modify this function and its interface to send a moveit_msgs.msg.CollisionObject instead of custom_msg.msg.Object + change its name to manipulate or something
        msg = Object(
            type = type,
            operation = operation,
            name = name,
            pose = pc.revert_from_vision(pose).publishable(),
            shape = Float64MultiArray(data=shape)
        )
        self.attach_object_pub.publish(msg)
    
    def detectionUpdated(self):
        return pt.DETECTION_UPDATED

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

    def goalAssignmentCallback(self, request: RequestHDGoal.Request, response: RequestHDGoal.Response) -> RequestHDGoal.Response:
        self.loginfo("Task executor received cmd")
        goal = request.goal
        is_abort = goal.target == HDGoal.ABORT
        if is_abort:
            self.abortTask()
            response.success = True
            response.message = HDGoal.OK
            return response
        if self.hasTask():
            self.loginfo("but already has task : ignoring")
            response.success = False
            response.message = HDGoal.ALREADY_HAS_GOAL
            return response
        if goal.target not in self.KNOWN_TASKS_NEW:
            self.loginfo("Unknown task")
            response.success = False
            response.message = HDGoal.UNKNOWN_GOAL
            return response

        possible_kwargs = {
            HDGoal.TOOL_PICKUP: {"tool": goal.tool},
            HDGoal.TOOL_PLACEBACK: {"tool": goal.tool},
            HDGoal.PREDEFINED_POSE: {"name": goal.predefined_pose},
        }
        kwargs = possible_kwargs.get(goal.target, {})
        self.task = self.KNOWN_TASKS_NEW[goal.target].select(self, **kwargs)
        self.new_task = True
        response.success = True
        response.message = HDGoal.OK
        return response
        
    def taskAssignmentCallback(self, msg: Task):
        """listens to /HD/fsm/task_assignment topic"""
        self.loginfo("Task executor received cmd")
        if self.hasTask():
            self.loginfo("but already has task : ignoring")
            return
        if msg.type not in self.KNOWN_TASKS:
            self.loginfo("Unknown task")
            return
        self.task = self.KNOWN_TASKS[msg.type].select(self)
        if msg.type == Task.NAMED_TARGET:
            self.task.addCommand(command.NamedJointTargetCommand(self, msg.str_slot))
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

    def abortCallback(self, msg: Int8):
        self.abortTask()
        
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
        self.task_outcome_pub.publish(msg)
        self.task = None

    def abortTask(self):
        """stops the assigned task"""
        if self.hasTask():
            self.task.abort()
            self.task = None
    
    def testVision(self):
        if len(pt.DETECTED_OBJECTS_POSE) == 0: return
        shape = [0.1, 0.2, 0.0001]
        pose = pt.DETECTED_OBJECTS_POSE[0].artag_pose
        name = "test_btn"
        self.addObjectToWorld(shape, pose, name)
        
    def testRockDetection(self):
        rd = pt.perception_tracker.rock_detection
        if rd.deprecated(): return
        shape = [rd.min_diameter, rd.max_diameter, rd.height]
        pose = pt.perception_tracker.rock_detection.rock_pose
        name = "test_rock"
        self.addObjectToWorld(shape, pose, name)

    def run(self):
        """main loop of the task executor"""
        rate = self.create_rate(25)   # 25hz
        while rclpy.ok():
            if self.new_task:
                # initiate the task in a new thread
                thread = threading.Thread(target=self.initiateTask)
                thread.start()
            # self.testVision()
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
