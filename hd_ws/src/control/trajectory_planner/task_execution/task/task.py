import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
from collections.abc import Callable
from task_execution.command import *
from kerby_interfaces.msg import Object


class CommandWrapper:
    def __init__(self):
        pass
        



class Task:
    """abstract class representing a task"""
    NONE_OPERATION = lambda cmd: None

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
        self.artag_pose = None
        self.object_pose = None

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

    def currentCommand(self):
        return self.command_chain[self.cmd_counter]
    
    def finished(self):
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

    def currentCommandValidated(self):
        """indicates after the request of execution of the command if the outcome is satisfactory and the next command can begin"""
        return self.currentCommand().done()

    def stopCondition(self):
        """indicates if task should be stopped because a command can't be executed"""
        return self.aborted or self.currentCommand().hasFailed()

    def oneCommandLoop(self):
        if self.aborted:
            return False
        self.currentCommand().execute()
        if self.stopCondition():
            return False
        return True

    def executeNextCommand(self):
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
        
    def execute(self):
        """executes all commands"""
        for _ in range(len(self.command_chain)):
            if not self.executeNextCommand():
                return False    # task failed
            time.sleep(self.pause_time)
        return True             # task succeeded

    def abort(self):
        """stops all movement"""
        self.aborted = True

    def scanForObjects(self):
        """try to get the pose of the ARtag and object for the task"""
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1:   # TODO: check if obj.id corresponds to task id here
                self.object_pose = obj.object_pose
                self.artag_pose = obj.artag_pose

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
