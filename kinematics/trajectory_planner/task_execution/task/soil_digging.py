from __future__ import annotations
from .task import *
from .tool_pickup import *


class DropSample(Task):
    def __init__(self, executor: Executor):
        super().__init__(executor)
    
    def constructCommandChain(self):
        super().constructCommandChain()

        self.addCommand(
            NamedJointTargetCommand("sample_retract"),
            description="retract to high position"
        )
        
        self.addCommand(
            NamedJointTargetCommand("above_sample_container"),
            description="go above sample container"
        )
        
        self.addCommand(
            NamedJointTargetCommand("sample_container_drop"),
            description="drop contents"
        )
