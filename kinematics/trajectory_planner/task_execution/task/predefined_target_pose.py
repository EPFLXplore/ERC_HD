from __future__ import annotations
from .task import *
from .tool_pickup import *


class PredefinedTargetPose(Task):
    def __init__(self, executor: Executor, name: str):
        super().__init__(executor)
        self.name = name
    
    def constructCommandChain(self):
        super().constructCommandChain()

        self.addCommand(NamedJointTargetCommand(name=self.name))
