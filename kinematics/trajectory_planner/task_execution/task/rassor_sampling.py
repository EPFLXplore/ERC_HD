from .task import *


class RassorSampling(Task):
    def __init__(self):
        self.ground_approach_steps = [0.2]  # [0.3, 0.2]

        self.j5_gripper_distance = 0.2      # distance between the axis of j5 and the tip of the gripper in the vertical coordinate when gripper is facing down
        self.j5_rassor_distance = 0.05      # distance between the axis of j5 and the furthest point of rassor in the vertical coordinate when gripper is facing forward
    
        self.sample_duration = 5    # [s]
        self.release_duration = 5   # [s]

        super().__init__()

    def getAdvanceDistance(self, target):
        return pt.DEPTH - target

    def getGroundContactDistance(self):
        return self.ground_approach_steps[-1] + self.j5_gripper_distance - self.j5_rassor_distance
    
    def constructCommandChain(self):
        super().constructCommandChain()

        # go to facing down pose
        self.addCommand(
            NamedJointTargetCommand(name="face_ground"),
            description = "look at ground"
        )

        # approach ground iteratively
        for target in self.ground_approach_steps:
            self.addCommand(
                StraightMoveCommand(axis=[0.0, 0.0, -1.0], distance=self.getAdvanceDistance(target)),
                pre_operation = lambda cmd: time.sleep(1.0),
                description = f"lower gripper to {target} meters above ground"
            )
        
        # turn gripper to make rassor face ground
        self.addCommand(
            JointSpaceCommand(mode=JointSpaceCmd.RELATIVE),
            pre_operation = lambda cmd: cmd.setJ5State(-math.pi/2),
            description = "turn gripper 90 degrees to make rassor face ground"
        )

        # make contact with ground with the rassor using cartesian path
        self.addCommand(
            StraightMoveCommand(axis=[0.0, 0.0, -1.0], distance=self.getGroundContactDistance()),
            description = "make contact with ground with the rassor"
        )

        # turn rassor
        self.addCommand(
            RassorCommand(action=RassorCommand.SAMPLE, duration=self.sample_duration),
            description = "rotate rassor to sample soil"
        )

        # go up some using cartesian path
        self.addCommand(
            StraightMoveCommand(axis=[0.0, 0.0, 1.0], distance=0.4),
            description = "go up in a straight line"
        )

        # go above science container
        self.addCommand(
            NamedJointTargetCommand(name="science_drop"),
            description = "go above science container"
        )
        
        # rotate rassor the opposite way to drop contents
        self.addCommand(
            RassorCommand(action=RassorCommand.RELEASE, duration=self.release_duration),
            description = "release rassor contents"
        )

        # go back to home
        self.addCommand(
            NamedJointTargetCommand(name="home"),
            description = "go back home to finilize task"
        )
