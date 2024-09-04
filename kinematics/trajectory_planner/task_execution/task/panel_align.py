from .task import *


class AlignPanel(Task):
    def __init__(self, executor):
        super().__init__(executor)
    
    def constructCommandChain(self):
        super().constructCommandChain()

        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scanForObjects(),
            description = "request detection"
        )
        self.addCommand(        # TODO: maybe disable collisions for this object
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(self.artag_pose),
                                        cmd.setShape([0.2, 0.1, 0.0001]),
                                        cmd.setName("artag")),
            description="add ARtag to world"
        )
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(qan.Pose(position=self.getScanPosition(),
                                                    orientation=self.getScanOrientation())),
            description = "go in front of ARtag"
        )
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scanForObjects(),
            description = "request new detection"
        )
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(qan.Pose(position=self.getScanPosition(),
                                                    orientation=self.getScanOrientation())),
            description = "go in front of ARtag again"
        )
