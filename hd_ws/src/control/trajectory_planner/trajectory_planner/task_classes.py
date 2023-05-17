import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import trajectory_planner.quaternion_arithmetic as qa
import trajectory_planner.pose_tracker as pt
import trajectory_planner.eef_pose_corrector as epc
from collections.abc import Callable


class Command(object):
    """abstract class representing a command"""
    def __init__(self, executor=None):
        self.executor = executor
        self.execute_count = 0

    def createSetter(self, attribute: str):
        """create a setter for the given attribute"""
        def setter(val):
            setattr(self, attribute, val)

        if not hasattr(self, "set" + attribute.capitalize()):
            setattr(self, "set" + attribute.capitalize(), setter)

    def createSetters(self, *attributes):
        for attr in attributes:
            self.createSetter(attr)

    def execute(self):
        """attempts to execute command
        returns a bool indicating if it succeeded"""
        self.execute_count += 1

    def abort(self):
        """stops all movement"""

class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, executor=None, pose=None, cartesian=False):
        super().__init__(executor)
        self.pose = pose    # Pose
        self.cartesian = cartesian
        self.createSetters("pose", "cartesian")
        self.finished = False

    def setPose(self, position=None, orientation=None, revert=True):
        if self.pose is None: 
            self.pose = Pose()
        if position is not None: 
            self.pose.position = position
        if orientation is not None: 
            self.pose.orientation = orientation
        if revert:
            self.pose = epc.revert(self.pose)

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, self.cartesian)
        self.finished = self.executor.waitForFeedback()


class StraightMoveCommand(Command):
    """moves the end effector in a straight line by a certain distance in a certain direction without affecting its orientation"""

    def __init__(self, executor=None, axis=(1,0,0), distance=0):
        super().__init__(executor)
        self.axis = axis
        self.distance = distance
        self.createSetters("axis", "distance")
    
    def setAxisFromOrientation(self, orientation: Quaternion, reverse=False):
        self.axis = qa.point_image([0.0, 0.0, 1.0], orientation)
        if reverse:
            self.axis = qa.mul(-1, self.axis)

    def constructPose(self):
        self.pose = Pose()
        self.pose.orientation = copy.deepcopy(pt.END_EFFECTOR_POSE.orientation)
        p = qa.list_to_point(qa.normalize(self.axis))
        p = qa.mul(self.distance, p)
        self.pose.position = qa.quat_to_point(qa.add(pt.END_EFFECTOR_POSE.position, p))

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        self.constructPose()
        super().execute()
        self.executor.sendPoseGoal(self.pose, True)
        self.finished = self.executor.waitForFeedback()


class GripperRotationCommand(Command):
    """rotates the gripper around the given axis by the given angle without affecting its position"""

    def __init__(self, executor=None, axis=(0,0,1), angle=0):
        super().__init__(executor)
        self.axis = axis
        self.angle = angle
        self.createSetters("axis", "angle")
    
    def constructPose(self):
        self.pose = Pose()
        self.pose.position = copy.deepcopy(pt.END_EFFECTOR_POSE.position)
        q = qa.quat(self.axis, self.angle)
        o = qa.mul(q, pt.END_EFFECTOR_POSE.orientation)
        self.pose.orientation = qa.quat_normalize(o)
    
    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, True)
        self.finished = self.executor.waitForFeedback()


class GripperManipulationCommand(Command):
    """opens/closes the gripper to a desired position"""
    def __init__(self, executor):
        super().__init__(executor)

    def execute(self):
        """publishes on /arm_control/joint_cmd topic for the motor controller"""
        super().execute()


class AddObjectCommand(Command):
    def __init__(self, executor=None, pose=None, shape=None, type="box", name="gustavo"):
        super().__init__(executor)
        self.pose = pose
        self.shape = shape
        self.type = type
        self.name = name
        self.createSetters("pose", "shape", "type", "name")
    
    def execute(self):
        super().execute()
        self.executor.addObjectToWorld(self.shape, self.pose, self.name, self.type)
        time.sleep(.5)


class RequestDetectionCommand(Command):
    def __init__(self, executor=None):
        super().__init__(executor)
        self.max_wait_time = 10
    
    def execute(self):
        super().execute()
        time.sleep(2)
        pt.deprecate_detection()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to leave release ressources
        while not pt.DETECTION_UPDATED:
            if time.time()-start > self.max_wait_time:
                return  # TODO: indicate that no detection was recorded (command failed)
            rate.sleep()


class Task(object):
    """abstract class representing a task"""

    def __init__(self, executor):
        self.executor = executor
        self.cmd_counter = 0
        self.command_chain = []
        self.pre_command_operation = []
        self.post_command_operation = []
        self.command_description = []
        self.constructCommandChain()
        self.aborted = False
        self.pause_time = 0

    def constructCommandChain(self):
        """constructs the chain of the commands that constitute the task"""

    def addCommand(self, command: Command, pre_operation: Callable = None, post_operation: Callable = None, description: str = ""):
        """add a new command to the command list"""
        if pre_operation is None:
            pre_operation = lambda cmd: None
        if post_operation is None:
            post_operation = lambda cmd: None

        command.executor = self.executor
        self.command_chain.append(command)
        self.pre_command_operation.append(pre_operation)
        self.post_command_operation.append(post_operation)
        self.command_description.append(description)

    def finished(self):
        """indicates if task has finished"""
    
    def update_world(self):
        """update the objects in the world"""

    def setupNextCommand(self):
        """gives the required information to the next command"""
        # deprecated
    
    def nextPreOperation(self):
        """execute the pre operation of the current command"""
        if self.pre_command_operation:
            self.pre_command_operation[self.cmd_counter](self.currentCommand())
    
    def nextPostOperation(self):
        """execute the post operation of the current command"""
        if self.post_command_operation:
            self.post_command_operation[self.cmd_counter](self.currentCommand())

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
        self.setupNextCommand()     # deprecated
        self.nextPreOperation()
        if not self.oneCommandLoop():
            return False
        while not self.currentCommandValidated():
            if not self.oneCommandLoop():
                return False
        
        self.nextPostOperation()
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


class PressButton2(Task):
    def __init__(self, executor, btn_id, pose=None, scan_pose=True):
        super().__init__(executor)
        self.btn_id = btn_id
        self.btn_pose = pose
        self.artag_pose = None
        self.scan_pose = scan_pose
        self.press_distance = 0.2
        self.pause_time = 2

    def scan_for_btn_pose(self):
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1 or obj.id == self.btn_id:
                relative_pose = obj.object_pose
                self.btn_pose = qa.compose_poses(epc.correct_eef_pose(), relative_pose)
                #self.btn_pose.orientation = qa.turn_around(self.btn_pose.orientation)
                self.artage_pose = self.btn_pose

    def currentCommand(self):
        return self.command_chain[self.cmd_counter]

    def getPressPosition(self, position=None):
        if position is None:
            position = self.btn_pose.position
        p = qa.point_image([0.0, 0.0, 1.0], self.btn_pose.orientation)
        d = 0.001
        if self.cmd_counter != 1:
            d += self.press_distance
        p = qa.mul(d, p)
        #p = qa.mul(-1, p)   # TODO: direction is reversed for some reason
        res = qa.quat_to_point(qa.add(position, p))
        return res

    def getPressOrientation(self):
        return qa.turn_around(self.btn_pose.orientation)

    def constructCommandChain(self):
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scan_for_btn_pose(),
            description = "request detection"
        )
        self.addCommand(
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(self.btn_pose),
                                         cmd.setShape([0.2, 0.1, 0.0001]),
                                         cmd.setName("btn")),
            description="add button to world"
        )
        self.addCommand(
            PoseCommand(self.executor),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(self.btn_pose.position),
                                                    orientation=self.getPressOrientation()),
            description = "go in front of button"
        )
        self.addCommand(
            StraightMoveCommand(),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation, reverse=True))
        )
        self.addCommand(
            StraightMoveCommand(),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation))
        )


class FlipSwitch(Task):
    def constructCommandChain(self):
        self.command_chain = [
            PoseCommand(self.executor),   # go at a predetermined position in front of the flip with gripper facing towards it
            StraightMoveCommand(self.executor),   # go forward enough to press the switch
            StraightMoveCommand(self.executor)   # go backwards
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

    def __init__(self, executor, axis=(0,0,0), velocity_scaling=1):
        """calls setNextGoal to set the first goal"""
        self.executor = executor
        self.axis = axis
        self.velocity_scaling = velocity_scaling
        self.max_step_distance = 0.1
        self.min_step_distance = 0.01
        self.pursue = True
        self.finished = False
    
    def get_distance(self):
        if self.velocity_scaling < 0.03:    # to eliminate ghost signals
            return 0
        return self.min_step_distance + (self.max_step_distance-self.min_step_distance)*self.velocity_scaling

    def execute(self):
        """executes all commands"""
        while self.pursue:
            self.pursue = False
            cmd = StraightMoveCommand(self.executor, axis=self.axis, distance=self.get_distance())
            cmd.constructPose()
            cmd.execute()
            self.pursue = False
        self.finished = True
    
    def abort(self):
        """stops all movement"""
        # TODO


class OrientationManualMotion:
    """special task allowing manual movement of the end effector (gripper) in a certain direction in a straight line with unchanging orientation"""

    def __init__(self, executor, axis=(0,0,0), velocity_scaling=1):
        """calls setNextGoal to set the first goal"""
        self.executor = executor
        self.axis = axis
        self.velocity_scaling = velocity_scaling
        self.max_step_angle = 1
        self.pursue = True
        self.finished = False
        
    def execute(self):
        """executes all commands"""
        while self.pursue:
            self.pursue = False
            cmd = GripperRotationCommand(self.executor, axis=self.axis, angle=self.max_step_angle)
            cmd.constructPose()
            cmd.execute()
            #time.sleep(.1)
        self.finished = True
    
    def abort(self):
        """stops all movement"""
        # TODO
