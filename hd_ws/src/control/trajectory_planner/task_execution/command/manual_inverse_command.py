from task_execution.command.command import *


class ManualInverseCommand(Command):
    def __init__(self, executor=None, axis=None):
        if axis is None:
            axis = [0, 0, 0]
        super().__init__(executor)
        self.axis = axis
    
    def execute(self):
        pass