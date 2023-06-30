import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
from collections.abc import Callable
from task_execution.command.all_commands import *


class Task:
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