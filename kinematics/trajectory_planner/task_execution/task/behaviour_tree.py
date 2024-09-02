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


class BTNode:
    def __init__(self, description: str = ""):
        self.description = description
        self.exec_result: bool = None
        
    def execute(self) -> bool:
        raise NotImplementedError
    
    def stopExecution(self):
        raise NotImplementedError
        
    def isBackground(self) -> bool:
        """
        Indicate whether the action of the node is a background operation.
        Used in the case of concurrent execution: once other actions have terminated,
        background operations should be stopped.
        """
        raise NotImplementedError
    
    def stashExecute(self):
        self.exec_result = self.execute()
    
    def getExecResult(self) -> bool:
        if self.exec_result is None:
            raise RuntimeError("Attempted to retrieve execution result, but no execution result has been stashed")
        return self.exec_result
    
    def __repr__(self) -> str:
        return type(self).__name__


class BTDecorator(BTNode):
    def __init__(self, node: BTNode):
        super().__init__(node.description)
        self.underlying_node = node
    
    def execute(self) -> bool:
        return self.underlying_node.execute
    
    def stopExecution(self):
        self.underlying_node.stopExecution()
        
    def isBackground(self) -> bool:
        self.underlying_node.isBackground()
    
    def stashExecute(self):
        self.exec_result = self.execute()
    
    def __repr__(self) -> str:
        return self.underlying_node.__repr__()


class Inverter(BTDecorator):
    def execute(self) -> bool:
        return not super().execute()


class Repeater(BTDecorator):
    def __init__(self, node: BTNode, count: int = 1):
        super().__init__(node)
        self.repeat_count = count
    
    def execute(self) -> bool:
        # arbitrary choice: return result of last execution
        for _ in range(self.repeat_count):
            res = super().execute()
        return res


def repeater(count: int = 1) -> Callable[[BTNode], Repeater]:
    return lambda node: Repeater(node, count)


class Succeeder(BTDecorator):
    def execute(self) -> bool:
        super().execute()
        return True


class Failer(BTDecorator):
    def execute(self) -> bool:
        super().execute()
        return False


class Cooldown(BTDecorator):
    def __init__(self, node: BTNode, seconds: float = 0.0):
        super().__init__(node)
        self.cooldown_seconds = seconds
    
    def execute(self) -> bool:
        time.sleep(self.cooldown_seconds)
        return super().execute()


def cooldown(seconds: float = 0.0) -> Callable[[BTNode], Cooldown]:
    return lambda node: Cooldown(node, seconds)


class ActionNode(BTNode):
    NONE_OPERATION: OPFunction = lambda cmd: None
    
    def __init__(self, command: Command, pre_operation: OPFunction = None, post_operation: OPFunction = None, description: str = ""):
        if pre_operation is None:
            pre_operation = ActionNode.NONE_OPERATION
        if post_operation is None:
            post_operation = ActionNode.NONE_OPERATION
            
        super().__init__(description)
        self.command = command
        self.pre_operation = pre_operation
        self.post_operation = post_operation
    
    def stopExecution(self):
        self.command.abort()
    
    def isBackground(self) -> bool:
        return self.command.isBackground()
    
    def _printUpdate(self):
        executor = get_executor()
        executor.loginfo("Next task command : " + self.description)
        
    def _currentCommandValidated(self) -> bool:
        """indicates after execution attempt of the command if it succeeded"""
        return self.command.done()

    def _stopCondition(self) -> bool:
        """indicates if command should stop being attempted because it can't be executed"""
        return self.command.hasFailed() # TODO: or aborted

    def _oneCommandLoop(self) -> bool:
        # TODO: if aborted return False
        # if self.aborted:
        #     return False
        self.command.execute()
        if self._stopCondition():
            return False
        return True

    def execute(self) -> bool:
        """attempts to execute the command
        returns a bool indicating if it succeeded"""
        self._printUpdate()
        self.pre_operation(self.command)
        while not self._currentCommandValidated():
            if not self._oneCommandLoop():
                return False        # command failed
        
        self.post_operation(self.command)
        return True
    
    def __repr__(self) -> str:
        return type(self.command).__name__


class CompositeNode(BTNode):
    def __init__(self, nodes: List[BTNode], description: str = ""):
        super().__init__(description)
        self.nodes = nodes
    
    def stopExecution(self):
        for node in self.nodes:
            node.stopExecution()
        
    def isBackground(self) -> bool:
        # maybe always return False for composite nodes
        return all(node.isBackground() for node in self.nodes)


class Sequence(CompositeNode):
    def execute(self) -> bool:
        for node in self.nodes:
            if not node.execute():
                return False
    
    def __repr__(self) -> str:
        return "->"


class Fallback(CompositeNode):
    def execute(self) -> bool:
        for node in self.nodes:
            if node.execute():
                return True
    
    def __repr__(self):
        return "?"


class ConcurrentNode(CompositeNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.child_exec_results: List[bool] = None
        
    def execute(self) -> bool:
        threads = [threading.Thread(target=node.stashExecute) for node in self.nodes]
         
        # start all threads
        for thread in threads:
            thread.start()
            
        # join threads corresponding to non background operations
        for node, thread in zip(self.nodes, threads):
            if not node.isBackground():
                thread.join()
        
        # stop all (other) threads
        for node in self.nodes:
            node.stopExecution()
        
        # retrieve exec results
        self.child_exec_results = [node.getExecResult() for node in self.nodes]
        
        return True     # by default, can be overriden


class SequenceConcurrent(ConcurrentNode):
    def execute(self) -> bool:
        super().execute()
        return all(self.child_exec_results)
    
    def __repr__(self):
        return "|||"
