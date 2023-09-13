from task_execution.command.command import *


class VoltmeterCommand(Command):
    EXTEND = 0
    RETRACT = 1
    EXTEND_ANGLE = 100.0
    RETRACT_ANGLE = 20.0

    def __init__(self, executor, action=None):
        super().__init__(executor)
        if action is None:
            action = self.EXTEND
        self.action = action
        self.wait_duration = 5.0

    def getAngle(self):
        return self.EXTEND_ANGLE if self.action == self.EXTEND else self.RETRACT_ANGLE
    
    def execute(self):
        super().execute()
        self.executor.sendVoltmeterCommand(self.getAngle())
        time.sleep(self.wait_duration)
