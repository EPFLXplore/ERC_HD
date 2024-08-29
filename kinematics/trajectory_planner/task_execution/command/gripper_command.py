from __future__ import annotations
from task_execution.command.command import *


class GripperCommand(Command):
    """Command for applying torque to the gripper for a certain duration"""

    OPEN = 0
    CLOSE = 1

    def __init__(self, executor=None, action: int = None, duration: float = float("inf"), torque_scaling_factor: float = 1.0):
        super().__init__(executor)
        if action is None:
            action = self.OPEN
        self.action = action
        self.duration = duration    # [s]
        self.torque_scaling_factor = torque_scaling_factor
        self.is_background = True
        self.stop_flag = False
    
    def abort(self):
        self.stop_flag = True
        
    def getTorqueSign(self):
        return 1 if self.action == self.OPEN else -1
    
    def execute(self):
        super().execute()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to release ressources
        while not(self.stop_flag) and time.time()-start < self.duration:
            self.executor.sendGripperTorque(self.getTorqueSign() * self.torque_scaling_factor)
            rate.sleep()
        self.executor.sendGripperTorque(0)  # stop the torque at the end


class GripperBackgroundCommand(BackgroundCommand):
    OPEN = 0
    CLOSE = 1

    def __init__(self, executor: Executor = None, action: int = None, torque_scaling_factor: float = 1.0):
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
        self.executor.sendGripperTorque(0)
