from .task import *
from .tool_pickup import *


class Dummy(Task):
    def __init__(self, executor):
        super().__init__(executor)
    
    def getAlignedPosition(self) -> Point:
        # align end effector with artag
        camera_pos = pc.CAMERA_TRANSFORM.position
        align_distance = 0.07
        # p = [camera_pos.x, camera_pos.y, align_distance]
        p = [0.0, 0.0, align_distance]
        return self.artag_pose.point_image(p)
    
    def constructCommandChain(self):
        super().constructCommandChain()

        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.5),
            pre_operation = lambda cmd: (cmd.setDistance(0.15),
                                         cmd.setAxisFromOrientation(pc.correct_gripper_pose().orientation, reverse=True)),
            description = "move back from button"
        )
        return
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scanForObjects(),
            description = "request detection"
        )
        self.addCommand(        # TODO: maybe disable collisions for this object
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(self.artag_pose),
                                        cmd.setShape([0.1, 0.2, 0.0001]),
                                        cmd.setName("artag")),
            description="add ARtag to world"
        )
        # self.addCommand(        # TODO: maybe disable collisions for this object
        #     AddObjectCommand(),
        #     pre_operation = lambda cmd: (cmd.setPose(qan.Pose(position=self.getAlignedPosition(),
        #                                             orientation=self.getScanOrientation())),
        #                                 cmd.setShape([0.1, 0.2, 0.0001]),
        #                                 cmd.setName("pose_to_reach")),
        #     description="add ARtag to world"
        # )
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(qan.Pose(position=self.getAlignedPosition(),
                                                    orientation=self.getScanOrientation())),
            description = "go in front of ARtag"
        )


DummyWithTools = combine_tasks(ToolPickup, Dummy, ToolPlaceback)
