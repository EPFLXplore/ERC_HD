from .task import *


class PressButton(Task):
    def __init__(self, executor, btn_id, pose=None):
        super().__init__(executor)
        self.btn_id = btn_id
        self.object_pose = pose
        self.artag_pose = None
        self.press_distance = 0.2
        self.scan_distance = 0.13        # from end effector in the z (forward if gripper is standardly oriented) coordinate
        self.pause_time = 0.5

    @property
    def btn_pose(self):
        return self.object_pose

    def getPressPosition(self):
        d = 0.001 + self.press_distance
        p = [0.0, 0.0, d]
        return qa.point_object_image(p, self.btn_pose)

    def getPressOrientation(self):
        return qa.turn_around(self.btn_pose.orientation)
    
    def getScanPosition(self):
        # give a position where the camera would be aligned with the ARtag
        # for now supposing camera has the same orientation as the end effector
        camera_pos = pc.CAMERA_TRANSFORM.position
        p = [camera_pos.y, -camera_pos.x, self.scan_distance]   # not sure why I need to exchange x and y here (x needs to be negated but I thing y doesn't although this hasn't been tested due to our y being 0)
        return qa.point_object_image(p, self.artag_pose)
    
    def getScanOrientation(self):
        return qa.turn_around(self.artag_pose.orientation)

    def constructCommandChain(self):
        super().constructCommandChain()

        self.constructStandardDetectionCommands("button", extended=True)

        self.addCommand(
            PoseCommand(self.executor),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(),
                                                    orientation=self.getPressOrientation()),
            description = "go in front of button"
        )
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.1),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation, reverse=True)),
            description = "click on button"
        )
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.1),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation)),
            description = "move back from button"
        )
        
        # TODO: maybe remove the objects added to the world
