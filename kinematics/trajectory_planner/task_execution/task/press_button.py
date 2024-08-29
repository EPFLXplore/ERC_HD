from .task import *


class PressButton(Task):
    def __init__(self, executor):
        super().__init__(executor)
        self.press_distance = 0.15
        self.scan_distance = 0.13        # from end effector in the local z coordinate (forward if gripper is standardly oriented)
        self.pause_time = 0.2

    @property
    def btn_pose(self):
        return self.object_pose

    def getPressPosition(self):
        d = 0.001 + self.press_distance
        p = [0.0, 0.0, d]
        return qa.point_object_image(p, self.btn_pose)

    def getPressOrientation(self):
        return qa.turn_around(self.btn_pose.orientation)

    def constructCommandChain(self):
        super().constructCommandChain()

        extended = True
        
        self.constructStandardDetectionCommands("button", extended=extended)
        
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(),
                                                    orientation=self.getPressOrientation()),
            description = "go in front of button"
        )
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.7),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation, reverse=True)),
            description = "click on button"
        )
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.7),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation)),
            description = "move back from button"
        )
        
        # TODO: maybe remove the objects added to the world
        #self.constructRemoveObjectsCommands("button", extended=extended)
