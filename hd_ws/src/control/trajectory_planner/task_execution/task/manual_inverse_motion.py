from task_execution.task.task import *


def normalize(v):
    n = math.sqrt(sum(x**2 for x in v))
    return [x/n for x in v]


def close(v1, v2, epsilon=10**-1):
    return sum((x-y)**2 for x, y in zip(v1, v2)) < epsilon**2


class ManualInverseMotion(Task):
    def __init__(self, executor):
        super().__init__(executor)
        self.axis = [0, 0, 0]
        self.command = None
    
    def updateAxis(self, axis):
        axis = normalize(axis)
        if self.command is not None and close(axis, self.axis):
            self.command.update()
            return
        self.command = 
    
    def execute(self):
        pass