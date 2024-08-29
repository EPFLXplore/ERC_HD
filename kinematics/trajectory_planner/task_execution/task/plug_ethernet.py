from .task import *


class EthernetApproach(Task):
    def __init__(self):
        super().__init__()
        self.press_distance = 0.2
    
    @property
    def ethernet_pose(self):
        return self.object_pose

    def getPressPosition(self):
        d = 0.001 + self.press_distance
        p = [0.0, 0.0, d]
        return qa.point_object_image(p, self.object_pose)

    def getPressOrientation(self):
        return qa.turn_around(self.object_pose.orientation)
    
    def constructCommandChain(self):
        super().constructCommandChain()

        self.constructStandardDetectionCommands(object_name="ethernet", extended=True)
        # approach
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(), orientation=self.getPressOrientation()),
            description="go in front of ethernet"
        )
