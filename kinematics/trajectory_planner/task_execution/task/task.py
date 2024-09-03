from __future__ import annotations
import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
import kinematics_utils.pose_tracker as pt
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
from collections.abc import Callable
from typing import Dict, List, Optional, Union
from task_execution.command import *
from custom_msg.msg import Object
from dataclasses import dataclass
from rclpy.timer import Rate
from typing import List, Dict, Callable


OPFunction = Callable[[Command], Any]


@dataclass(frozen=True)
class CommandData:
    command: Command
    pre_operation: OPFunction
    post_operation: OPFunction
    description: str


@dataclass(frozen=True)
class BackgroundCommandData:
    command: BackgroundCommand
    description: str


class Task:
    # TODO: add support for merging tasks
    # TODO: add behaviour tree structure
    """abstract class representing a task"""
    NONE_OPERATION: OPFunction = lambda cmd: None
    BACKGROUND_COMMANDS: Dict[str, BackgroundCommandData] = {}

    def __init__(self, executor: Executor, construct_command_chain: bool = True):
        self.executor = executor
        self.cmd_counter = 0
        self.command_chain: List[CommandData] = []
        # self.background_commands: Dict[str, BackgroundCommandData] = {}
        self.executor.logwarn(f"init {type(self).__name__}")
        if construct_command_chain: self.constructCommandChain()
        self.aborted = False
        self.pause_time = 0.0
        self.artag_pose = qan.Pose()
        self.object_pose = qan.Pose()
        self.scan_distance = 0.13   # from end effector in the local z coordinate (forward if gripper is standardly oriented)

    def constructCommandChain(self):
        """constructs the chain of the commands that constitute the task"""

    def addCommand(self, command: Command, pre_operation: Optional[OPFunction] = None, post_operation: Optional[OPFunction] = None, description: str = ""):
        """add a new command to the command list"""
        if pre_operation is None:
            pre_operation = Task.NONE_OPERATION
        if post_operation is None:
            post_operation = Task.NONE_OPERATION

        command.executor = self.executor
        self.command_chain.append(CommandData(command, pre_operation, post_operation, description))
    
    def declareBackgroundCommand(self, id: str, command: BackgroundCommand, description: str = ""):
        """
        Declare a background command. This doesn't set any start or stop point for the command, 
        the setCommandStartPoint and setCommandStopPoint methods need to be called for that (after this method has been called).
        """
        if id in self.BACKGROUND_COMMANDS:
            # TODO: deal with the case when id already exists : raise an error or maybe just do nothing
            return  # do nothing for now
        command.executor = self.executor
        self.BACKGROUND_COMMANDS[id] = BackgroundCommandData(command, description)
    
    def setBackgrounCommandActionPoint(self, id: str, action: int, pre_operation: Optional[OPFunction] = None, post_operation: Optional[OPFunction] = None, description: str = "", allow_lazy_cmd_retrieval: bool = False):
        """
        Set the starting or stopping point of the BackgroundCommand identified by id at the current point in the workflow.
        """
        if not allow_lazy_cmd_retrieval and id not in self.BACKGROUND_COMMANDS:
            raise KeyError(f"'{id}' is not a valid background command for this task")
        if description == "":
            description = f"{'Starting' if action == BackgroundCommand.START else 'Stopping'} background command '{self.BACKGROUND_COMMANDS[id].description}'"
        
        WrapperClass = BackgroundCommandStart if action == BackgroundCommand.START else BackgroundCommandStop
        command = WrapperClass(lambda: self.BACKGROUND_COMMANDS[id].command)
        self.addCommand(command, pre_operation, post_operation, description)

    def setBackgroundCommandStartPoint(self, id: str, pre_operation: Optional[OPFunction] = None, post_operation: Optional[OPFunction] = None, description: str = "", allow_lazy_cmd_retrieval: bool = False):
        """
        Set the starting point of the BackgroundCommand identified by id at the current point in the workflow.
        If the given description is empty, it will be replace by "Starting background command '{description of the background command}".
        """
        self.setBackgrounCommandActionPoint(id, BackgroundCommand.START, pre_operation, post_operation, description, allow_lazy_cmd_retrieval)
    
    def setBackgroundCommandStopPoint(self, id: str, pre_operation: Optional[OPFunction] = None, post_operation: Optional[OPFunction] = None, description: str = "", allow_lazy_cmd_retrieval: bool = False):
        """
        Set the stoping point of the BackgroundCommand identified by id at the current point in the workflow.
        If the given description is empty, it will be replace by "Stoping background command '{description of the background command}'".
        """
        self.setBackgrounCommandActionPoint(id, BackgroundCommand.STOP, pre_operation, post_operation, description, allow_lazy_cmd_retrieval)

    def finished(self) -> bool:
        """indicates if task has finished"""
        return self.cmd_counter == len(self.command_chain)
    
    @property
    def _currentCommandData(self) -> CommandData:
        return self.command_chain[self.cmd_counter]
    
    @property
    def _currentCommand(self) -> Command:
        return self._currentCommandData.command
    
    @property
    def _currentPreOperation(self) -> OPFunction:
        return self._currentCommandData.pre_operation
    
    @property
    def _currentPostOperation(self) -> OPFunction:
        return self._currentCommandData.post_operation
    
    @property
    def _currentCommandDescription(self) -> str:
        return self._currentCommandData.description
    
    def _printUpdate(self):
        self.executor.loginfo("Next task command : " + self._currentCommandDescription)
    
    def _nextPreOperation(self):
        """execute the pre operation of the current command"""
        self._currentPreOperation(self._currentCommand)
    
    def _nextPostOperation(self):
        """execute the post operation of the current command"""
        self._currentPostOperation(self._currentCommand)

    def _currentCommandValidated(self) -> bool:
        """indicates after the request of execution of the command if the outcome is satisfactory and the next command can begin"""
        return self._currentCommand.done()

    def _stopCondition(self) -> bool:
        """indicates if task should be stopped because a command can't be executed"""
        return self.aborted or self._currentCommand.hasFailed()

    def _oneCommandLoop(self) -> bool:
        if self.aborted:
            return False
        self._currentCommand.execute()
        if self._stopCondition():
            return False
        return True

    def _executeNextCommand(self) -> bool:
        """attempts to execute next command on the command chain
        returns a bool indicating if it succeeded"""
        self._printUpdate()
        self._nextPreOperation()
        while not self._currentCommandValidated():
            if not self._oneCommandLoop():
                return False        # command failed
        
        self._nextPostOperation()
        self.cmd_counter += 1
        return True                 # command succeeded
        
    def execute(self, terminate: bool = True) -> bool:
        """executes all commands"""
        for _ in range(len(self.command_chain)):
            if not self._executeNextCommand():
                self._terminate()
                return False    # task failed
            time.sleep(self.pause_time)
        if terminate: self._terminate()
        return True             # task succeeded
    
    def _getActiveBackgroundCommands(self) -> Dict[str, BackgroundCommandData]:
        alive_cmds = {}
        for cmd_id, cmd_data in self.BACKGROUND_COMMANDS.items():
            if cmd_data.command.isAlive():
                alive_cmds[cmd_id] = cmd_data
        return alive_cmds
    
    def _terminate(self):
        """
        make sure execution finishes gracefully
        cleanup background commands that have been killed
        """
        for id, cmd_data in list(self.BACKGROUND_COMMANDS.items()):
            if not cmd_data.command.isAlive():
                self.BACKGROUND_COMMANDS.pop(id)
    
    def _old_terminate(self, wait: bool = True):
        """make sure execution finishes gracefully"""
        # deprecated: background commands are now class attributes and shouldn't be killed at the end of individual task execution
        for cmd_data in self.BACKGROUND_COMMANDS.values():
            if cmd_data.command.isAlive(): cmd_data.command.softStop(wait=False)
        if not wait: return
        rate: Rate = self.executor.create_rate(25)
        while any(cmd_data.command.isAlive() for cmd_data in self.BACKGROUND_COMMANDS.values()):
            rate.sleep()

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
                self.object_pose = qan.Pose.make(obj.object_pose)
                self.artag_pose = qan.Pose.make(obj.artag_pose)

    def getScanPosition(self) -> Point:
        # give a position where the camera would be aligned with the ARtag
        # for now supposing camera has the same orientation as the end effector
        camera_pos = pc.CAMERA_TRANSFORM.position
        p = [-camera_pos.x, camera_pos.y, self.scan_distance]
        return self.artag_pose.point_image(p)
        p = [camera_pos.y, -camera_pos.x, self.scan_distance]   # not sure why I need to exchange x and y here (x needs to be negated but I think y doesn't although this hasn't been tested due to our y being 0)
        return qa.point_object_image(p, self.artag_pose)
    
    def getScanOrientation(self) -> qan.Quaternion:
        return self.artag_pose.orientation.turn_around()
        return qa.turn_around(self.artag_pose.orientation)
    
    def constructStandardDetectionCommands(self, object_name: str = "object", object_box: Union[tuple, list] = (0.1, 0.2, 0.0001), extended: bool = True, add_objects: bool = True):
        """an example of a series of commands for accurate detection of ARtag and associated object"""
        if extended:
            self.addCommand(
                RequestDetectionCommand(),
                post_operation = lambda cmd: self.scanForObjects(),
                description = "request detection"
            )
            if add_objects:
                self.addCommand(        # TODO: maybe disable collisions for this object
                    AddObjectCommand(),
                    pre_operation = lambda cmd: (cmd.setPose(self.artag_pose),
                                                cmd.setShape([0.1, 0.2, 0.0001]),
                                                cmd.setName("artag")),
                    description="add ARtag to world"
            )
            self.addCommand(
                PoseCommand(),
                pre_operation = lambda cmd: cmd.setPose(qan.Pose(position=self.getScanPosition(),
                                                        orientation=self.getScanOrientation())),
                description = "go in front of ARtag"
            )
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scanForObjects(),
            description = "request new detection"
        )
        if add_objects:
            self.addCommand(        # TODO: maybe disable collisions for this object
                AddObjectCommand(),
                pre_operation = lambda cmd: (cmd.setPose(self.object_pose),
                                            cmd.setShape(list(object_box)),
                                            cmd.setName(object_name)),
                description=f"add {object_name} to world"
            )

    def constructRemoveObjectsCommands(self, object_name: str = "object", extended: bool = True):
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

    def addObjectAxisCommand(self, object_name: str = "object"):
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
    
    def constructOpenGripperCommands(self, high_torque: float = 1.0, low_torque: float = 0.1, post_completion_wait: float = 0.0):
        self.addCommand(
            GripperCommand(GripperCommand.OPEN, duration=1.0, torque_scaling_factor=high_torque),
            description = "open gripper high torque"
        )
        self.addCommand(
            GripperCommand(GripperCommand.OPEN, duration=3.0, torque_scaling_factor=low_torque),
            post_operation = lambda cmd: time.sleep(post_completion_wait),
            description = "open gripper low torque"
        )
    
    def constructCloseGripperCommands(self, high_torque: float = 1.0, low_torque: float = 0.1, post_completion_wait: float = 0.0):
        self.addCommand(
            GripperCommand(GripperCommand.CLOSE, duration=1.0, torque_scaling_factor=high_torque),
            description = "close gripper high torque"
        )
        self.addCommand(
            GripperCommand(GripperCommand.CLOSE, duration=3.0, torque_scaling_factor=low_torque),
            post_operation = lambda cmd: time.sleep(post_completion_wait),
            description = "close gripper low torque"
        )



def combine_tasks(*tasks: Type[Task]) -> Type[Task]:
    class CombinedTask(Task):
        def __init__(self, executor: Executor):
            super().__init__(executor)
            self.tasks = [task_type(executor) for task_type in tasks]
        
        def execute(self) -> bool:
            for task in self.tasks:
                success = task.execute(terminate=False)
                if not success:
                    task._terminate()
                    return False
            task._terminate()
            return True

    return CombinedTask
