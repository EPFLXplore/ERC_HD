import sys
import rospy
import time
import math
import copy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from task_execution.srv import PoseGoal, JointGoal
import task_execution.quaternion_arithmetic as qa


END_EFFECTOR_POSE = geometry_msgs.msg.Pose()
#pose_goal_pub = rospy.Publisher("/arm_control/pose_goal", PoseGoal, queue_size=5)
#joint_goal_pub = rospy.Publisher("/arm_control/joint_goal", JointGoal, queue_size=5)


def register_end_effector_pose(pose):
    """listens to /arm_control/end_effector_pose topic"""
    global END_EFFECTOR_POSE
    END_EFFECTOR_POSE = pose


class Command:
    """abstract class representing a command"""

    def finished(self):
        """indicates if the command can be considered as executed"""
    def execute(self):
        """attempts to execute command
        returns a bool indicating if it succeeded"""
    def abort(self):
        """stops all movement"""

class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, pose=None, cartesian=False):
        self.pose = pose    # geometry_msgs.msg.Pose
        self.cartesian = cartesian
        self.finished = False

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        rospy.wait_for_service('/arm_control/pose_goal')
        try:
            proxy = rospy.ServiceProxy('/arm_control/pose_goal', PoseGoal)
            cmd_id = 0  # TODO: increment id at each command
            resp = proxy(cmd_id, self.pose, self.cartesian)
            self.finished = resp.ok
        except rospy.ServiceException as e:
            # TODO: handle the exception (maybe)
            print("Service call failed: %s"%e)


class StraightMoveCommand(Command):
    """moves the end effector in a straight line by a certain distance in a certain direction without affecting its orientation"""

    def __init__(self, axis=(1,0,0), distance=0):
        self.axis = axis
        self.distance = distance
        
    def constructPose(self):
        self.pose = geometry_msgs.msg.Pose()
        self.pose.orientation = copy.deepcopy(END_EFFECTOR_POSE.orientation)
        p = qa.list_to_point(qa.normalize(self.axis))
        p = qa.mul(self.distance, p)
        self.pose.position = qa.quat_to_point(qa.add(END_EFFECTOR_POSE.position, p))

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        rospy.wait_for_service('/arm_control/pose_goal')
        try:
            proxy = rospy.ServiceProxy('/arm_control/pose_goal', PoseGoal)
            cmd_id = 0  # TODO: increment id at each command
            resp = proxy(cmd_id, self.pose, True)
            self.finished = resp.ok
        except rospy.ServiceException as e:
            # TODO: handle the exception (maybe)
            print("Service call failed: %s"%e)


class GripperRotationCommand(Command):
    """rotates the gripper around the given axis by the given angle without affecting its position"""

    def __init__(self, axis=(0,0,1), angle=0):
        self.axis = axis
        self.angle = angle
    
    def constructPose(self):
        self.pose = geometry_msgs.msg.Pose()
        self.pose.position = copy.deepcopy(END_EFFECTOR_POSE.position)
        q = qa.quat(self.axis, self.angle)
        o = qa.mul(q, END_EFFECTOR_POSE.orientation)
        self.pose.orientation = qa.quat_normalize(o)
    
    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        rospy.wait_for_service('/arm_control/pose_goal')
        try:
            proxy = rospy.ServiceProxy('/arm_control/pose_goal', PoseGoal)
            cmd_id = 0  # TODO: increment id at each command
            resp = proxy(cmd_id, self.pose, False)
            self.finished = resp.ok
        except rospy.ServiceException as e:
            # TODO: handle the exception (maybe)
            print("Service call failed: %s"%e)


class GripperManipulationCommand(Command):
    """opens/closes the gripper to a desired position"""
    def execute(self):
        """publishes on /arm_control/joint_cmd topic for the motor controller"""


class Task(object):
    """abstract class representing a task"""

    def __init__(self):
        self.cmd_counter = 0
        self.command_chain = []
        self.constructCommandChain()
        self.aborted = False
        self.pause_time = 0

    def constructCommandChain(self):
        """constructs the chain of the commands that constitue the task"""

    def finished(self):
        """indicates if task has finished"""
    
    def update_world(self):
        """update the objects in the world"""

    def setupNextCommand(self):
        """gives the required information to the next command"""

    def currentCommandValidated(self):
        """indicates after the request of execution of the command if the outcome is satisfactory and the next command can begin"""
        return True

    def stopCondition(self):
        """indicates if task should be stopped because a command can't be executed"""
        return False

    def oneCommandLoop(self):
        self.command_chain[self.cmd_counter].execute()
        if self.stopCondition():
            return False
        return True

    def executeNextCommand(self):
        """attempts to execute next command on the command chain
        returns a bool indicating if it succeeded"""
        self.setupNextCommand()
        if not self.oneCommandLoop():
            return False
        while not self.currentCommandValidated():
            if not self.oneCommandLoop():
                return False
        print("NEXT COMMAAAAAAND")
        self.cmd_counter += 1
        return True
        
    def execute(self):
        """executes all commands"""
        for _ in range(len(self.command_chain)):
            self.executeNextCommand()
            time.sleep(self.pause_time)
    
    def abort(self):
        """stops all movement"""
        # TODO


class PressButton(Task):
    def __init__(self, btn_pose):
        super(PressButton, self).__init__()
        self.btn_pose = btn_pose
        self.press_distance = 0.1
        self.pause_time = 2
    
    def currentCommand(self):
        print(len(self.command_chain))
        print(self.cmd_counter)
        return self.command_chain[self.cmd_counter]

    def getPressPosition(self):
        p = qa.point_image([0, 0, 1], self.btn_pose.orientation)
        d = 0.001
        if self.cmd_counter != 1:
            d += self.press_distance
        p = qa.mul(d, p)
        p = qa.mul(-1, p)   # TODO: direction is reversed for some reason
        res = qa.quat_to_point(qa.add(self.btn_pose.position, p))
        return res
    
    def getPressOrientation(self):
        # for now
        q = qa.quat([0,0,1], math.pi)
        return qa.mul(self.btn_pose.orientation, q)

    def setupNextCommand(self):
        cmd = self.currentCommand()
        """if self.cmd_counter == 0:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
        elif self.cmd_counter == 1:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.orientation = self.getPressOrientation()
            cmd.pose.position = self.getPressPosition()
            cmd.cartesian = True
        elif self.cmd_counter == 2:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
            cmd.cartesian = True"""
        if self.cmd_counter == 0:
            cmd.pose = geometry_msgs.msg.Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
        elif self.cmd_counter == 1:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            cmd.constructPose()
        elif self.cmd_counter == 2:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()

    def constructCommandChain(self):
        """self.command_chain = [
            PoseCommand(),
            PoseCommand(),
            PoseCommand()
        ]"""
        self.command_chain = [
            PoseCommand(),   # go at a predetermined position in front of the button with gripper facing towards it
            StraightMoveCommand(),   # go forward enough to press the button
            StraightMoveCommand()   # go backwards
        ]

class FlipSwitch(Task):
    def constructCommandChain(self):
        command_chain = [
            PoseCommand(),   # go at a predetermined position in front of the flip with gripper facing towards it
            StraightMoveCommand(),   # go forward enough to press the switch
            StraightMoveCommand()   # go backwards
        ]

"""class RotateSwitch(Task):

class PickUpCable(Task):

class PlugInCable(Task):

class PickUpProbe(Task):

class PlaceProbeOnContainer(Task):

class PickProbeFromContainer(Task):

class InsertProbeInSoil(Task):

class PickUpJumper(Task):

class PutJumperOnPins(Task):

class GoToHomePosition(Task):"""


class PositionManualMotion:
    """special task allowing manual movement of the end effector (gripper) in a certain direction in a straight line with unchanging orientation"""

    def __init__(self, axis=(0,0,0), velocity_scaling=1):
        """calls setNextGoal to set the first goal"""
        self.axis = axis
        self.velocity_scaling = velocity_scaling
        self.max_step_distance = 0.05
        self.pursue = True
        self.finished = False
        
    def execute(self):
        """executes all commands"""
        while self.pursue:
            self.pursue = False
            cmd = StraightMoveCommand(axis=self.axis, distance=self.max_step_distance)
            cmd.constructPose()
            cmd.execute()
            self.pursue = False
        self.finished = True
    
    def abort(self):
        """stops all movement"""
        # TODO


class OrientationManualMotion:
    """special task allowing manual movement of the end effector (gripper) in a certain direction in a straight line with unchanging orientation"""

    def __init__(self, axis=(0,0,0), velocity_scaling=1):
        """calls setNextGoal to set the first goal"""
        self.axis = axis
        self.velocity_scaling = velocity_scaling
        self.max_step_angle = 1
        self.pursue = True
        self.finished = False
        
    def execute(self):
        """executes all commands"""
        while self.pursue:
            self.pursue = False
            cmd = GripperRotationCommand(axis=self.axis, angle=self.max_step_angle)
            cmd.constructPose()
            cmd.execute()
            #rospy.sleep(.1)
        self.finished = True
    
    def abort(self):
        """stops all movement"""
        # TODO
