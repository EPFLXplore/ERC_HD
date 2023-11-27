from .command import *
from hd_interfaces.msg import JointSpaceCmd


class JointSpaceCommand(Command):
    def __init__(self, executor=None, mode=JointSpaceCmd.RELATIVE, states: list = None):
        super().__init__(executor)
        if states is None:
            states = [0.0]*7
        self.mode = mode
        self.states = states
        # for i in range(6):
        #     setattr(self, f"setJ{i+1}State", lambda state: self.setState(i, state))
    
    def setToCurrentState(self):
        self.states = [JointSpaceCmd.CURRENT_STATE]*7

    def setState(self, joint_index: int, state: float):
        self.states[joint_index] = state
    
    def setStates(self, states: list):
        self.states = states

    def setJ5State(self, state: float):
        self.states[4] = state

    def execute(self):
        super().execute()
        self.executor.sendJointSpaceCmd(self.mode, self.states)
        success = self.executor.waitForFeedback()
        if not success:
            self.has_failed = False
