from __future__ import annotations
from task_execution.command.command import *


class NamedJointTargetCommand(Command):
    def __init__(self, name: str = "zero"):
        super().__init__()
        self.name = name
    
    def execute(self):
        super().execute()
        self.executor.sendNamedJointTarget(self.name)
        self.finished = self.executor.waitForFeedback()
        if not self.finished:
            self.has_failed = True
            self.fail_message = "couldn't reach predefined pose"
