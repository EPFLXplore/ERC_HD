from .command import *


class BackgroundGripperCommand(BackgroundCommand):
    """Command for applying torque to the gripper for a certain duration"""

    OPEN = 0
    CLOSE = 1

    def __init__(self, executor=None, action: int=None, torque_scaling_factor: float=1.0):
        super().__init__(executor)
        if action is None:
            action = self.OPEN
        self.action = action
        self.torque_scaling_factor = torque_scaling_factor
    
    def getTorqueSign(self):
        return 1 if self.action == self.OPEN else -1
    
    def _executeCycle(self):
        self.executor.sendGripperTorque(self.getTorqueSign() * self.torque_scaling_factor)

    def _cleanup(self):
        self.executor.sendGripperTorque(0)  # stop the torque at the end
