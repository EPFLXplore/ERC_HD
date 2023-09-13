from .task import *


class RockSampling(Task):
    def __init__(self, executor):
        super().__init__(executor)
        self.rise_distance = 0.5      # after picking the rock, before going to a default position
    
    def constructCommandChain(self):
        super().constructCommandChain()
        # only dropping the rock, the detection and picking part is manual for now
        self.constructDropCommands()
    
    def constructDropCommands(self):
        # go up some using cartesian path
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.7),
            pre_operation = lambda cmd: (cmd.setDistance(self.rese_distance),
                                         cmd.setAxis([0.0, 0.0, 1.0])),
            description = "rise up"
        )

        self.addCommand(
            NamedJointTargetCommand(name="zero"),
            description = "go to zero position"
        )

        # go above science container
        self.addCommand(
            NamedJointTargetCommand(name="science_drop"),
            description = "go above science container"
        )

        self.addCommand(
            GripperCommand(action=GripperCommand.OPEN, duration=3),
            description = "open gripper to drop rock"
        )

        self.addCommand(
            GripperCommand(action=GripperCommand.CLOSE, duration=4, torque_scaling_factor=0.5),
            description = "close gripper"
        )

        self.addCommand(
            NamedJointTargetCommand(name="home"),
            description = "go back to home"
        )

