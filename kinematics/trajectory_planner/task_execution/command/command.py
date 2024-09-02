from __future__ import annotations
import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_tracker as pt
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
from collections.abc import Callable
from typing import Any, Type
import threading
from rclpy.node import Node
from rclpy.timer import Rate
from typing import TYPE_CHECKING, List, Dict, Callable
import task_execution
# import task_execution.task_executor
if TYPE_CHECKING:   # fake import, only for annotations
    from task_execution.task_executor import Executor


class Observations:
    INSTANCE: Observations = None
    
    def __new__(cls):
        if cls.INSTANCE is not None:
            raise RuntimeError("Observations class can only have one instance")
        instance = super().__new__(cls)
        cls.INSTANCE = instance
        return instance
    
    @classmethod
    def get_instance(cls) -> Observations:
        return cls.INSTANCE
    
    def __init__(self):
        self.object_pose = Pose()
        self.artag_pos = Pose()
    

def get_executor() -> Executor:
    return task_execution.task_executor.Executor()


class Command:
    """abstract class representing a command"""

    def __init__(self):
        self.executor: Executor = None
        self.execute_count = 0
        self.has_failed = False
        self.is_background = False

    def createSetter(self, attribute: str):
        """create a setter for the given attribute"""
        def setter(val: Any):
            setattr(self, attribute, val)
        
        def camelCasify(s: str) -> str:
            if len(s) == 0:
                return ""
            camel_s = ""
            next_capital = True
            for c in s:
                if c == "_":
                    next_capital = True
                    continue
                camel_s += c.capitalize() if next_capital else c
                next_capital = False
            return camel_s

        setter_name = "set" + camelCasify(attribute)
        if not hasattr(self, setter_name):
            setattr(self, setter_name, setter)

    def createSetters(self, *attributes: str):
        for attr in attributes:
            self.createSetter(attr)

    def isBackground(self) -> bool:
        return self.is_background
    
    def execute(self):
        """attempts to execute command"""
        self.execute_count += 1

    def abort(self):
        """stops all movement"""

    def done(self) -> bool:
        """indicate if command has executed correctly (by default returns true as soon as the commands has been executed once)"""
        return self.execute_count > 0

    def hasFailed(self) -> bool:
        return self.has_failed


class BackgroundCommand:
    START = 0
    STOP = 1

    def __init__(self):
        self.executor: Executor = None
        self._has_started = False
        self._has_finished = False
        self._stop_flag = False
        self.execution_thread = threading.Thread(target=self.execute)
        self.rate: Rate = None
    
    def isAlive(self) -> bool:
        return self.execution_thread.is_alive()
    
    @property
    def has_finished(self) -> bool:
        return self._has_finished and not self.isAlive()
    
    def setRate(self, hz: int):
        self.rate = self.executor.create_rate(hz)

    def start(self):
        self._has_started = True
        self.execution_thread.start()

    def softStop(self, wait: bool = True):
        self._stop_flag = True
        if wait:
            self.execution_thread.join()
        self._has_finished = True
    
    def hardStop(self):     # TODO: figure out how to forcefully stop the thread or delete this method
        raise NotImplementedError("BackgroundCommand.hardStop method is not implemented")

    def _executeCycle(self):
        """one cyle of the execute loop: override this method, not execute"""
        pass

    def _cleanup(self):
        """called before ending the execution thread"""
        pass

    def execute(self):
        self.setRate(25)
        while not self._stop_flag:
            self._executeCycle()
            self.rate.sleep()
        self._cleanup()



LazyEvalBackgroundCommand = Callable[[], BackgroundCommand]


class BackgroundCommandActionPoint(Command):
    def __init__(self, background_command: LazyEvalBackgroundCommand):
        """lazy evaluation on the background_command argument for technical purposes"""
        super().__init__()
        self._background_command = background_command
    
    @property
    def background_command(self) -> BackgroundCommand:
        return self._background_command()


class BackgroundCommandStart(BackgroundCommandActionPoint):
    def execute(self):
        super().execute()
        self.background_command.start()


class BackgroundCommandStop(BackgroundCommandActionPoint):
    def __init__(self, background_command: LazyEvalBackgroundCommand, wait: bool = True):
        super().__init__(background_command)
        self.wait = wait
    
    def execute(self):
        super().execute()
        self.background_command.softStop(wait=self.wait)
