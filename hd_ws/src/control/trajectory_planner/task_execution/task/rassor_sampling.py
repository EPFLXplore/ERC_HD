from .task import *


class RassorSampling(Task):
    def __init__(self, executor):
        super().__init__(executor)
        self.ground_approach_steps = [0.3, 0.2]
    
    def getAdvanceDistance(self, target):
        return pt.DEPTH - target
    
    def constructCommandChain(self):
        super().constructCommandChain()

        # go to facing down pose
        self.addCommand(
            NamedJointTargetCommand(name="facing_ground"),
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
            
        )

        # make contact with ground with the rassor using cartesian path
        # turn rassor
        # go up some using cartesian path
        # go to home or something
        # go above science container
        # rotate rassor the opposite way to drop contents
        # go back to home
