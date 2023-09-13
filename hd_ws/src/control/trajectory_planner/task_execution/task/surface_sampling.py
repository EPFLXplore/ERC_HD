from .task import *


class SurfaceSampling(Task):
    def __init__(self, executor):
        super().__init__(executor)
    
    def constructCommandChain(self):
        super().constructCommandChain()

        # go to facing down pose
        self.addCommand(
            NamedJointTargetCommand(name="facing_ground"),
            description = "look at ground"
        )

        # approach ground iteratively
        
        
        # turn gripper to make rassor face ground
        # make contact with ground with the rassor using cartesian path
        # turn rassor
        # go up some using cartesian path
        # go to home or something
        # go above science container
        # rotate rassor the opposite way to drop contents
        # go back to home
