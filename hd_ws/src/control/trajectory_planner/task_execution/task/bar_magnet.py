from .task import *


class BarMagnetApproach(Task):
    ARTAG_BAR_TRANSFORM = Pose(
        position = Point(x=0.0, y=-0.9, z=0.15),
        orientation = qa.quat([1.0, 0.0, 0.0], -math.pi/2)
    )

    def __init__(self, executor):
        super().__init__(executor)
        self.above_distance = 0.3
    
    @property
    def bar_pose(self):
        return self.object_pose
    
    def scanForObjects(self):
        """try to get the pose of the ARtag and object for the task"""
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1:   # TODO: check if obj.id corresponds to task id here
                self.artag_pose = obj.artag_pose
                self.object_pose = qa.compose_poses(self.artag_pose, self.ARTAG_BAR_TRANSFORM)

    def getAboveBarPosition(self):
        d = self.above_distance
        p = [0.0, 0.0, d]
        return qa.point_object_image(p, self.bar_pose)

    def getBarOrientation(self):
        return qa.turn_around(self.bar_pose.orientation)
    
    def constructCommandChain(self):
        super().constructCommandChain()

        # detection
        self.constructStandardDetectionCommands("metal_bar", extended=False)
        self.addControlPanelCommand()
        # self.addObjectAxisCommand("metal_bar")

        # approach
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(position=self.getAboveBarPosition(), orientation=self.getBarOrientation()),
            description="go above metal bar"
        )
