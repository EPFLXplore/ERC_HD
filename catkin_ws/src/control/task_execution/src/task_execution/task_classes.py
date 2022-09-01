import rospy
import time
import math
import copy
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from task_execution.srv import PoseGoal, JointGoal
from task_execution.msg import Object
import task_execution.quaternion_arithmetic as qa
import task_execution.pose_tracker as pt


#pose_goal_pub = rospy.Publisher("/arm_control/pose_goal", PoseGoal, queue_size=5)
#joint_goal_pub = rospy.Publisher("/arm_control/joint_goal", JointGoal, queue_size=5)
add_object_pub = rospy.Publisher("/arm_control/add_object", Object, queue_size=5)


class Command(object):
    """abstract class representing a command"""
    def __init__(self):
        self.execute_count = 0

    def execute(self):
        """attempts to execute command
        returns a bool indicating if it succeeded"""
        self.execute_count += 1

    def abort(self):
        """stops all movement"""

class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, pose=None, cartesian=False):
        super(PoseCommand, self).__init__()
        self.pose = pose    # geometry_msgs.msg.Pose
        self.cartesian = cartesian
        self.finished = False

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super(PoseCommand, self).execute()
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
        super(StraightMoveCommand, self).__init__()
        self.axis = axis
        self.distance = distance
        
    def constructPose(self):
        self.pose = geometry_msgs.msg.Pose()
        self.pose.orientation = copy.deepcopy(pt.END_EFFECTOR_POSE.orientation)
        p = qa.list_to_point(qa.normalize(self.axis))
        p = qa.mul(self.distance, p)
        self.pose.position = qa.quat_to_point(qa.add(pt.END_EFFECTOR_POSE.position, p))

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super(StraightMoveCommand, self).execute()
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
        super(GripperRotationCommand, self).__init__()
        self.axis = axis
        self.angle = angle
    
    def constructPose(self):
        self.pose = geometry_msgs.msg.Pose()
        self.pose.position = copy.deepcopy(pt.END_EFFECTOR_POSE.position)
        q = qa.quat(self.axis, self.angle)
        o = qa.mul(q, pt.END_EFFECTOR_POSE.orientation)
        self.pose.orientation = qa.quat_normalize(o)
    
    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super(GripperRotationCommand, self).execute()
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
    def __init__(self):
        super(GripperManipulationCommand, self).__init_()

    def execute(self):
        """publishes on /arm_control/joint_cmd topic for the motor controller"""
        super(GripperManipulationCommand, self).execute()


class AddObjectCommand(Command):
    def __init__(self, pose, dims, type="box", name=""):
        super(AddObjectCommand, self).__init__()
        self.pose = pose
        self.dims = Float32MultiArray()
        self.dims.data = dims
        self.type = type
        self.name = name
    
    def execute(self):
        super(AddObjectCommand, self).execute()
        msg = Object()
        msg.type = self.type
        msg.name = self.name
        msg.pose = self.pose
        msg.dims = self.dims
        add_object_pub.publish(msg)
        rospy.sleep(.5)


class RequestDetectionCommand(Command):
    def __init__(self):
        super(RequestDetectionCommand, self).__init__()
        self.max_wait_time = 10
    
    def execute(self):
        super(RequestDetectionCommand, self).execute()
        rospy.sleep(1.5)
        pt.deprecate_detection()
        start = time.time()
        while not pt.DETECTION_UPDATED:
            if time.time()-start > self.max_wait_time:
                return


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
        self.press_distance = 0.2
        self.pause_time = 2
    
    def currentCommand(self):
        print(len(self.command_chain))
        print(self.cmd_counter)
        return self.command_chain[self.cmd_counter]

    def getPressPosition(self):
        p = qa.point_image([0, 0, 1], self.btn_pose.orientation)
        d = self.press_distance + 0.001
        p = qa.mul(d, p)
        #p = qa.mul(-1, p)   # TODO: direction is reversed for some reason
        res = qa.quat_to_point(qa.add(self.btn_pose.position, p))
        return res
    
    def getPressOrientation(self):
        # for now
        #q = qa.quat([0,0,1], math.pi)
        #return qa.mul(self.btn_pose.orientation, q)
        axis = qa.point_image([1,0,0], self.btn_pose.orientation)
        q = qa.quat(axis, math.pi)
        return qa.mul(q, self.btn_pose.orientation)
        return qa.inv(self.btn_pose.orientation)
        return qa.mul(qa.inv(self.btn_pose.orientation), q) #qa.inv(qa.mul(self.btn_pose.orientation, q))

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
            cmd.axis = qa.mul(-1, cmd.axis)
            cmd.constructPose()
        elif self.cmd_counter == 2:
            cmd.distance = self.press_distance
            cmd.axis = qa.point_image([0, 0, 1], self.btn_pose.orientation)
            #cmd.axis = qa.mul(-1, cmd.axis)
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
        self.max_step_distance = 0.1
        self.min_step_distance = 0.01
        self.pursue = True
        self.finished = False
    
    def get_distance(self):
        if self.velocity_scaling < 0.01:    # to eliminate ghost signals
            return
        return self.min_step_distance + (self.max_step_distance-self.min_step_distance)*self.velocity_scaling

    def execute(self):
        """executes all commands"""
        while self.pursue:
            self.pursue = False
            cmd = StraightMoveCommand(axis=self.axis, distance=self.get_distance())
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
