import time
import math
import copy
from geometry_msgs.msg import Pose
import trajectory_planner.quaternion_arithmetic as qa
import trajectory_planner.pose_tracker as pt
import trajectory_planner.eef_pose_corrector as epc


class Command(object):
    """abstract class representing a command"""
    def __init__(self, executor):
        self.executor = executor
        self.execute_count = 0

    def execute(self):
        """attempts to execute command
        returns a bool indicating if it succeeded"""
        self.execute_count += 1

    def abort(self):
        """stops all movement"""

class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, executor, pose=None, cartesian=False):
        super().__init__(executor)
        self.pose = pose    # Pose
        self.cartesian = cartesian
        self.finished = False

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, self.cartesian)
        self.finished = self.executor.waitForFeedback()


class StraightMoveCommand(Command):
    """moves the end effector in a straight line by a certain distance in a certain direction without affecting its orientation"""

    def __init__(self, executor, axis=(1,0,0), distance=0):
        super().__init__(executor)
        self.axis = axis
        self.distance = distance
        
    def constructPose(self):
        self.pose = Pose()
        self.pose.orientation = copy.deepcopy(pt.END_EFFECTOR_POSE.orientation)
        p = qa.list_to_point(qa.normalize(self.axis))
        p = qa.mul(self.distance, p)
        self.pose.position = qa.quat_to_point(qa.add(pt.END_EFFECTOR_POSE.position, p))

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, True)
        self.finished = self.executor.waitForFeedback()


class GripperRotationCommand(Command):
    """rotates the gripper around the given axis by the given angle without affecting its position"""

    def __init__(self, executor, axis=(0,0,1), angle=0):
        super().__init__(executor)
        self.axis = axis
        self.angle = angle
    
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
    def __init__(self, executor, pose=None, shape=None, type="box", name="gustavo"):
        super().__init__(executor)
        self.pose = pose
        self.shape = shape
        self.type = type
        self.name = name
    
    def execute(self):
        super().execute()
        self.executor.addObjectToWorld(self.shape, self.pose, self.name, self.type)
        time.sleep(.5)


class RequestDetectionCommand(Command):
    def __init__(self, executor):
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
        self.constructCommandChain()
        self.aborted = False
        self.pause_time = 0

    def constructCommandChain(self):
        """constructs the chain of the commands that constitute the task"""

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
    def __init__(self, executor, btn_id, pose=None, scan_pose=True):
        super().__init__(executor)
        self.btn_id = btn_id
        self.btn_pose = pose
        self.artag_pose = None
        self.scan_pose = scan_pose
        self.press_distance = 0.2
        self.pause_time = 2
    
    def scan_for_btn_pose(self):
        # RequestDetectionCommand(self.executor).execute()
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1 or obj.id == self.btn_id:
                relative_pose = obj.object_pose
                self.btn_pose = qa.compose_poses(pt.END_EFFECTOR_POSE, relative_pose)
                self.btn_pose.orientation = qa.turn_around(self.btn_pose.orientation)
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

    def setupNextCommand(self):
        #self.executor.logwarn("STARTING CMD " + str(self.cmd_counter) + " : " + self.command_description[self.cmd_counter])
        cmd = self.currentCommand()
        if self.cmd_counter == 1:
            if 1 or self.scan_pose:
                self.scan_for_btn_pose()
            #self.artag_pose = copy.deepcopy(self.btn_pose)
            #self.artag_pose.y += 0.15
            cmd.pose = self.artag_pose    # TODO: don't give self.artag_pose if it is None and abort the add object command
            cmd.shape = [0.2, 0.1, 0.0001]
            cmd.name = "AR_tag"
        elif self.cmd_counter == 2:
            cmd.pose = Pose()
            cmd.pose.position = self.getPressPosition(self.artag_pose.position)
            cmd.pose.orientation = self.getPressOrientation()
        elif self.cmd_counter == 4:
            if self.scan_pose:
                self.scan_for_btn_pose()
            cmd.pose = self.btn_pose
            cmd.shape = [0.2, 0.1, 0.0001]
            cmd.name = "btn"
        elif self.cmd_counter == 5:
            cmd.pose = Pose()
            cmd.pose.position = self.getPressPosition(self.btn_pose.position)
            cmd.pose.orientation = self.getPressOrientation()
        elif self.cmd_counter == 6:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()
        elif self.cmd_counter == 7:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            #cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()
        #if self.cmd_counter > 2:
        #    cmd.cartesian = True
        """elif self.cmd_counter == 1:
            cmd.pose = Pose()
            cmd.pose.orientation = self.getPressOrientation()
            cmd.pose.position = self.getPressPosition()
            cmd.cartesian = True
        elif self.cmd_counter == 2:
            cmd.pose = Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
            cmd.cartesian = True"""

        """if self.cmd_counter == 0:
            cmd.pose = Pose()
            cmd.pose.position = self.getPressPosition()
            cmd.pose.orientation = self.getPressOrientation()
        elif self.cmd_counter == 1:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()
        elif self.cmd_counter == 2:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            #cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()"""

    def constructCommandChain(self):
        self.command_chain = [
            RequestDetectionCommand(self.executor),
            AddObjectCommand(self.executor),
            PoseCommand(self.executor),   # go at a predetermined position in front of the button with gripper facing towards it
            RequestDetectionCommand(self.executor),
            AddObjectCommand(self.executor),
            PoseCommand(self.executor),
            StraightMoveCommand(self.executor),   # go forward enough to press the button
            StraightMoveCommand(self.executor)   # go backwards
        ]
        self.command_description = [
            "request detection",
            "add AR tag to world",
            "go in front of AR tag",
            "request detection",
            "add button to world",
            "go in front of button",
            "move straight to button",
            "move staight away from button"
        ]


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

    def setupNextCommand(self):
        self.executor.loginfo("STARTING CMD " + str(self.cmd_counter) + " : " + self.command_description[self.cmd_counter])
        cmd = self.currentCommand()
        if self.cmd_counter == 0:
             if self.scan_pose:
                self.scan_for_btn_pose()
        elif self.cmd_counter == 1:
            if self.scan_pose:
                self.scan_for_btn_pose()
            cmd.pose = self.btn_pose
            cmd.shape = [0.2, 0.1, 0.0001]
            cmd.name = "btn"
        elif self.cmd_counter == 2:
            cmd.pose = Pose()
            cmd.pose.position = self.getPressPosition(self.btn_pose.position)
            cmd.pose.orientation = self.getPressOrientation()
            cmd.pose = epc.revert(cmd.pose)
        elif self.cmd_counter == 3:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0.0, 0.0, 1.0], self.btn_pose.orientation)
            cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()
        elif self.cmd_counter == 4:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0.0, 0.0, 1.0], self.btn_pose.orientation)
            #cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()

    def constructCommandChain(self):
        self.command_chain = [
            RequestDetectionCommand(self.executor),
            AddObjectCommand(self.executor),
            PoseCommand(self.executor),   # go at a predetermined position in front of the button with gripper facing towards it
            StraightMoveCommand(self.executor),   # go forward enough to press the button
            StraightMoveCommand(self.executor)   # go backwards
        ]
        self.command_description = [
            "request detection",
            "add button to world",
            "go in front of button",
            "move straight to button",
            "move staight away from button"
        ]


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
