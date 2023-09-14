#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from task_execution.task import PressButton, PlugVoltmeterAlign, PlugVoltmeterApproach, RassorSampling, RockSampling, BarMagnet
import task_execution.task
from task_execution.command import NamedJointTargetCommand
from kerby_interfaces.msg import Task, Object, PoseGoal, JointSpaceCmd
from hd_interfaces.msg import TargetInstruction
from avionics_interfaces.msg import ServoRequest, ServoResponse
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64MultiArray, Int8, String, UInt32
from motor_control_interfaces.msg import MotorCommand
import threading
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
import kinematics_utils.quaternion_arithmetic as qa


class Executor(Node):
    def __init__(self):
        super().__init__("kinematics_task_executor")
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

        self.task = None    # usually a Task from task_execution.task but could also be just a Command
        self.new_task = False

        self.traj_feedback_update = False
        self.traj_feedback = False

        self.received_voltmeter_response = False

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
    
    def waitForVoltmeterResponse(self, timeout=1, hz=10):
        self.received_voltmeter_response = False
        rate = self.create_rate(hz)
        start = time.time()
        while not self.received_voltmeter_response:
            if time.time()-start > timeout:
                return False
            rate.sleep()
        return True

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

    def sendPoseGoal(self, goal: Pose, cartesian=False, velocity_scaling_factor=1.0):
        msg = PoseGoal(
            goal = goal,
            cartesian = cartesian,
            velocity_scaling_factor = velocity_scaling_factor
        )
        self.pose_target_pub.publish(msg)
    
    def sendJointGoal(self, goal: list):
        msg = Float64MultiArray(data=goal)
        self.joint_target_pub.publish(msg)
    
    def sendGripperTorque(self, torque_scaling_factor):
        msg = MotorCommand(
            name = "Gripper",
            mode = 2,   # MotorCommand.TORQUE,
            command = torque_scaling_factor
        )
        self.motor_command_pub.publish(msg)
    
    def sendRassorTorque(self, torque_scaling_factor):
        msg = MotorCommand(
            name = "Rassor",
            mode = 2,   # MotorCommand.TORQUE,
            command = torque_scaling_factor
        )
        self.motor_command_pub.publish(msg)

    def sendVoltmeterCommand(self, angle):
        msg = ServoRequest(destination_id=0, channel=1, angle=angle)
        self.voltmeter_pub.publish(msg)

    def sendJointSpaceCmd(self, mode, states: list):
        msg = JointSpaceCmd(
            mode=mode,
            states=Float64MultiArray(data=states)
        )
        self.joint_space_cmd_pub.publish(msg)

    def addObjectToWorld(self, shape: list, pose: Pose, name: str, type=Object.BOX, operation=Object.ADD):
        msg = Object()
        msg.type = type
        msg.operation = operation
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
    
    def voltmeterResponseCallback(self, msg: ServoResponse):
        if msg.success:
            self.received_voltmeter_response = True

    def taskAssignementCallback(self, msg: Task):
        """listens to /HD/fsm/task_assignment topic"""
        self.loginfo("Task executor received cmd")
        if self.hasTask():
            return
        if msg.type == Task.BUTTON:
            self.loginfo("Button task")
            self.task = PressButton(self, msg.id, msg.pose)
        elif msg.type == Task.PLUG_VOLTMETER_ALIGN:
            self.loginfo("Plug voltmeter task")
            self.task = PlugVoltmeterAlign(self)
        elif msg.type == Task.PLUG_VOLTMETER_APPROACH:
            self.loginfo("Plug voltmeter task")
            self.task = PlugVoltmeterApproach(self)
        elif msg.type == Task.NAMED_TARGET:
            self.loginfo("Named target task")
            self.task = task_execution.task.Task(self)
            self.task.addCommand(NamedJointTargetCommand(self, msg.str_slot))
        elif msg.type == Task.PICK_ROCK :
            self.loginfo("Rock smpling task")
            self.task = RockSampling(self)
        elif msg.type == Task.RASSOR_SAMPLE:
            self.loginfo("Rassor sample task")
            self.task = RassorSampling(self)
        elif msg.type == Task.ETHERNET_CABLE:
            self.loginfo("Plug ethernet task")
            # TODO
        else:
            return
        
        self.new_task = True
    
    def CSMaintenanceCallback(self, msg: Int8):
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
