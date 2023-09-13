from task_execution.command.command import *


class VoltmeterCommand(Command):
    EXTEND = 0
    RETRACT = 1

    def __init__(self, executor, action=None):
        super().__init__(executor)
        if action is None:
            action = self.EXTEND
        self.action = action
        self.wait_duration = 5.0

    def execute(self):
        super().execute()
        self.executor.sendVoltmeterCommand(self.action)
        time.sleep(self.wait_duration)
