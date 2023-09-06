from task_execution.task.task import *


class PressButton(Task):
    def __init__(self, executor, btn_id, pose=None, scan_pose=True):
        super().__init__(executor)
        self.btn_id = btn_id
        self.btn_pose = pose
        self.artag_pose = None
        self.scan_pose = scan_pose
        self.press_distance = 0.2
        self.pause_time = 2

    def scan_for_btn_pose(self):
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1 or obj.id == self.btn_id:
                self.btn_pose = obj.object_pose
                self.artag_pose = obj.artag_pose

    def getPressPosition(self, position=None):
        if position is None:
            position = self.btn_pose.position
        p = qa.point_image([0.0, 0.0, 1.0], self.btn_pose.orientation)
        d = 0.001
        if self.cmd_counter != 1:
            d += self.press_distance
        p = qa.mul(d, p)
        res = qa.quat_to_point(qa.add(position, p))
        return res

    def getPressOrientation(self):
        return qa.turn_around(self.btn_pose.orientation)

    def constructCommandChain(self):
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scan_for_btn_pose(),
            description = "request detection"
        )
        self.addCommand(
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(self.btn_pose),
                                         cmd.setShape([0.2, 0.1, 0.0001]),
                                         cmd.setName("btn")),
            description="add button to world"
        )
        self.addCommand(
            PoseCommand(self.executor),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(self.btn_pose.position),
                                                    orientation=self.getPressOrientation()),
            description = "go in front of button"
        )
        self.addCommand(
            StraightMoveCommand(),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation, reverse=True))
        )
        self.addCommand(
            StraightMoveCommand(),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.btn_pose.orientation))
        )
