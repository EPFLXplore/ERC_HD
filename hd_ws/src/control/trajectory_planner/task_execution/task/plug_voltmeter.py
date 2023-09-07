from task_execution.task.task import *


class PlugVoltmeter(Task):
    def __init__(self, executor, pose):
        super().__init__(executor)
        self.pose = pose