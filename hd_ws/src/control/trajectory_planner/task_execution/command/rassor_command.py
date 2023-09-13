from task_execution.command.command import *


class RassorCommand(Command):
    SAMPLE = 0
    RELEASE = 1

    def __init__(self, executor=None, action=None, duration=1.0, torque_scaling_factor=1.0):
        super().__init__(executor)
        if action is None:
            action = self.SAMPLE
        self.action = action
        self.duration = duration    # [s]
        self.torque_scaling_factor = torque_scaling_factor
    
    def getTorqueSign(self):
        # TODO: ensure this is the right direction
        return 1 if self.action == self.SAMPLE else -1
    
    def execute(self):
        super().execute()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to release ressources
        while not time.time()-start < self.duration:
            self.executor.sendRassorTorque(self.getTorqueSign() * self.torque_scaling_factor)
            rate.sleep()
