from __future__ import annotations
from task_execution.command.command import *


class NamedJointTargetCommand(Command):
    def __init__(self, name: str = "zero"):
        super().__init__()
        self.name = name
    
    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendNamedJointTarget(self.name)
        self.finished = self.executor.waitForFeedback()
