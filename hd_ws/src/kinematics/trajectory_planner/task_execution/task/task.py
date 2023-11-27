import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
from collections.abc import Callable
from typing import Dict, List
from task_execution.command import *
from hd_interfaces.msg import Object


Executor = Executor     # ici ça code (visual studio doesn't find Executor so I redeclare it here so that there is no ugly underline in the rest)


class BackgroundCommandWrapper:
    def __init__(self, command: BackgroundCommand, description: str):
        self.command = command
        self.description = description


class Task:
    """abstract class representing a task"""
    NONE_OPERATION = lambda cmd: None

    def __init__(self, executor):
        self.executor: Executor = executor
        self.cmd_counter = 0
        self.command_chain: List[Command] = []
        self.background_commands: Dict[str, BackgroundCommandWrapper] = {}
        self.pre_command_operation: List[Callable] = []
        self.post_command_operation: List[Callable] = []
        self.command_description: List[str] = []
        self.constructCommandChain()
        self.aborted = False
        self.pause_time = 0
        self.artag_pose = Pose()
        self.object_pose = Pose()
        self.scan_distance = 0.13   # from end effector in the local z coordinate (forward if gripper is standardly oriented)

    def constructCommandChain(self):
        """constructs the chain of the commands that constitute the task"""

    def addCommand(self, command: Command, pre_operation: Callable = None, post_operation: Callable = None, description: str = ""):
        """add a new command to the command list"""
        if pre_operation is None:
            pre_operation = Task.NONE_OPERATION
        if post_operation is None:
            post_operation = Task.NONE_OPERATION

        command.executor = self.executor
        self.command_chain.append(command)
        self.pre_command_operation.append(pre_operation)
        self.post_command_operation.append(post_operation)
        self.command_description.append(description)
    
    def declareBackgroundCommand(self, id: str, command: BackgroundCommand, description: str = ""):
        """
        Declare a background command. This doesn't set any start or stop point for the command, 
        the setCommandStartPoint and setCommandStopPoint methods need to be called for that (after this method has been called).
        """
        # TODO: deal with the case when id already exists : raise an error or maybe just do nothing 
        self.background_commands[id] = BackgroundCommandWrapper(command, description)
    
    def setCommandStartPoint(self, id: str, pre_operation: Callable = None, post_operation: Callable = None, description: str = ""):
        """
        Set the starting point of the BackgroundCommand identified by id at the current point in the workflow.
        If the given description is empty, it will be replace by "Starting background command '{description of the background command}".
        """
        if id not in self.background_commands:
            raise KeyError(f"'{id}' is not a valid background command for this task")
        if pre_operation is None:
            pre_operation = Task.NONE_OPERATION
        if post_operation is None:
            post_operation = Task.NONE_OPERATION
        if description == "":
            description = f"Starting background command '{self.background_commands[id].description}'"
        
        command = BackgroundCommandStart(self.executor, self.background_commands[id].command)
        self.command_chain.append(command)
        self.pre_command_operation.append(pre_operation)
        self.post_command_operation.append(post_operation)
        self.command_description.append(description)
    
    def setCommandStopPoint(self, id: str, pre_operation: Callable = None, post_operation: Callable = None, description: str = ""):
        """
        Set the stoping point of the BackgroundCommand identified by id at the current point in the workflow.
        If the given description is empty, it will be replace by "Stoping background command '{description of the background command}'".
        """
        if id not in self.background_commands:
            raise KeyError(f"'{id}' is not a valid background command for this task")
        if pre_operation is None:
            pre_operation = Task.NONE_OPERATION
        if post_operation is None:
            post_operation = Task.NONE_OPERATION
        if description == "":
            description = f"Stoping background command '{self.background_commands[id].description}'"
        
        command = BackgroundCommandStop(self.executor, self.background_commands[id].command)
        self.command_chain.append(command)
        self.pre_command_operation.append(pre_operation)
        self.post_command_operation.append(post_operation)
        self.command_description.append(description)

    def currentCommand(self) -> Command:
        return self.command_chain[self.cmd_counter]
    
    def finished(self) -> bool:
        """indicates if task has finished"""
        return self.cmd_counter == len(self.command_chain)
    
    def update_world(self):
        """update the objects in the world"""
        # TODO: probably remove this function, don't remember what I intended it for
    
    def printUpdate(self):
        self.executor.loginfo("Next task command : " + self.command_description[self.cmd_counter])
    
    def nextPreOperation(self):
        """execute the pre operation of the current command"""
        if self.pre_command_operation:
            self.pre_command_operation[self.cmd_counter](self.currentCommand())
    
    def nextPostOperation(self):
        """execute the post operation of the current command"""
        if self.post_command_operation:
            self.post_command_operation[self.cmd_counter](self.currentCommand())

    def currentCommandValidated(self) -> bool:
        """indicates after the request of execution of the command if the outcome is satisfactory and the next command can begin"""
        return self.currentCommand().done()

    def stopCondition(self) -> bool:
        """indicates if task should be stopped because a command can't be executed"""
        return self.aborted or self.currentCommand().hasFailed()

    def oneCommandLoop(self) -> bool:
        if self.aborted:
            return False
        self.currentCommand().execute()
        if self.stopCondition():
            return False
        return True

    def executeNextCommand(self) -> bool:
        """attempts to execute next command on the command chain
        returns a bool indicating if it succeeded"""
        self.printUpdate()
        self.nextPreOperation()
        while not self.currentCommandValidated():
            if not self.oneCommandLoop():
                return False        # command failed
        
        self.nextPostOperation()
        self.cmd_counter += 1
        return True                 # command succeeded
        
    def execute(self) -> bool:
        """executes all commands"""
        for _ in range(len(self.command_chain)):
            if not self.executeNextCommand():
                return False    # task failed
            time.sleep(self.pause_time)
        return True             # task succeeded

    def abort(self):
        """stops all movement"""
        self.aborted = True

    def getObjectPose(self) -> Pose:
        return self.object_pose

    def getARTagPose(self) -> Pose:
        return self.artag_pose
    
    def scanForObjects(self):
        """try to get the pose of the ARtag and object for the task"""
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1:   # TODO: check if obj.id corresponds to task id here
                self.object_pose = obj.object_pose
                self.artag_pose = obj.artag_pose

    def getScanPosition(self) -> Point:
        # give a position where the camera would be aligned with the ARtag
        # for now supposing camera has the same orientation as the end effector
        camera_pos = pc.CAMERA_TRANSFORM.position
        p = [camera_pos.y, -camera_pos.x, self.scan_distance]   # not sure why I need to exchange x and y here (x needs to be negated but I thing y doesn't although this hasn't been tested due to our y being 0)
        return qa.point_object_image(p, self.artag_pose)
    
    def getScanOrientation(self) -> Quaternion:
        return qa.turn_around(self.artag_pose.orientation)
    
    def constructStandardDetectionCommands(self, object_name="object", object_box=(0.2, 0.1, 0.0001), extended=True):
        """an example of a series of commands for accurate detection of ARtag and associated object"""
        if extended:
            self.addCommand(
                RequestDetectionCommand(),
                post_operation = lambda cmd: self.scanForObjects(),
                description = "request detection"
            )
            self.addCommand(        # TODO: maybe disable collisions for this object
                AddObjectCommand(),
                pre_operation = lambda cmd: (cmd.setPose(self.artag_pose),
                                            cmd.setShape([0.2, 0.1, 0.0001]),
                                            cmd.setName("artag")),
                description="add ARtag to world"
            )
            self.addCommand(
                PoseCommand(self.executor),
                pre_operation = lambda cmd: cmd.setPose(position=self.getScanPosition(),
                                                        orientation=self.getScanOrientation()),
                description = "go in front of ARtag"
            )
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scanForObjects(),
            description = "request new detection"
        )
        self.addCommand(        # TODO: maybe disable collisions for this object
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(self.object_pose),
                                         cmd.setShape(list(object_box)),
                                         cmd.setName(object_name)),
            description=f"add {object_name} to world"
        )

    def constructRemoveObjectsCommands(self, object_name="object", extended=True):
        if extended:
            self.addCommand(
                AddObjectCommand(name="artag", operation=Object.REMOVE)
            )
        self.addCommand(
            AddObjectCommand(name=object_name, operation=Object.REMOVE)
        )
        self.addCommand(
            AddObjectCommand(name=f"{object_name}_axis", operation=Object.REMOVE)
        )
        self.addCommand(
            AddObjectCommand(name="control_panel", operation=Object.REMOVE)
        )

    def addControlPanelCommand(self):
        self.addCommand(
            AddObjectCommand(name="control_panel"),
            pre_operation = lambda cmd: (cmd.setPose(self.getARTagPose()),
                                         cmd.setShape([1.0, 1.0, 0.0001])),
            description = "add control panel to world"
        )

    def addObjectAxisCommand(self, object_name: str):
        self.addCommand(
            AddObjectCommand(name=f"{object_name}_axis", operation=Object.ADD),
            pre_operation = lambda cmd: (cmd.setPose(
                                                        Pose(
                                                            position = qa.make_point(qa.add(self.object_pose.position, qa.mul(0.05, qa.point_image([0.0, 0.0, 1.0], self.object_pose.orientation)))),
                                                            orientation = self.object_pose.orientation
                                                        )
                                                    ),
                                         cmd.setShape([0.01, 0.01, 0.1])
                                        ),
            description = "add axis of the object to visualize orientation"
        )
    
    def constructOpenGripperCommands(self, high_torque=1.0, low_torque=0.1, post_completion_wait=0.0):
        self.addCommand(
            GripperCommand(self, GripperCommand.OPEN, duration=1.0, torque_scaling_factor=high_torque),
            description = "open gripper high torque"
        )
        self.addCommand(
            GripperCommand(self, GripperCommand.OPEN, duration=3.0, torque_scaling_factor=low_torque),
            post_operation = lambda cmd: time.sleep(post_completion_wait),
            description = "open gripper low torque"
        )
    
    def constructCloseGripperCommands(self, high_torque=1.0, low_torque=0.1, post_completion_wait=0.0):
        self.addCommand(
            GripperCommand(self, GripperCommand.CLOSE, duration=1.0, torque_scaling_factor=high_torque),
            description = "close gripper high torque"
        )
        self.addCommand(
            GripperCommand(self, GripperCommand.CLOSE, duration=3.0, torque_scaling_factor=low_torque),
            post_operation = lambda cmd: time.sleep(post_completion_wait),
            description = "close gripper low torque"
        )
