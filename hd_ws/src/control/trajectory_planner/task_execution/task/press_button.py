from task_execution.task.task import *


BTN = 0
ARTAG = 1


class PressButton(Task):
    def __init__(self, executor, btn_id, pose=None, scan_pose=True):
        super().__init__(executor)
        self.btn_id = btn_id
        self.btn_pose = pose
        self.artag_pose = None
        self.scan_pose = scan_pose
        self.press_distance = 0.2
        self.scan_distance = 0.2        # from end effector in the z (forward) coordinate
        self.pause_time = 2

    def scan_for_btn_pose(self):
        while pt.DETECTED_OBJECTS_LOCKED:
            pass
        for obj in pt.DETECTED_OBJECTS_POSE:
            if 1 or obj.id == self.btn_id:
                self.btn_pose = obj.object_pose
                self.artag_pose = obj.artag_pose

    def getPressPosition(self):
        d = 0.001
        if self.cmd_counter != 1:
            d += self.press_distance
        p = [0.0, 0.0, d]
        return qa.point_object_image(p, self.btn_pose)

    def getPressOrientation(self):
        return qa.turn_around(self.btn_pose.orientation)
    
    def getScanPosition(self):
        # for now supposing camera has the same orientation as the end effector
        p = qa.scalar_mul(-1, pc.CAMERA_TRANSFORM.position)
        p = qa.point_add(p, [0.0, 0.0, self.scan_distance])
        return qa.point_object_image(p, self.artag_pose)
    
    def getScanOrientation(self):
        return qa.turn_around(self.artag_pose.orientation)

    def constructCommandChain(self):
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scan_for_btn_pose(),
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
            PoseCommand(self.executor),
            pre_operation = lambda cmd: cmd.setPose(position=self.getScanPosition(),
                                                    orientation=self.getScanOrientation()),
            description = "go in front of ARtag"
        )
        self.addCommand(
            RequestDetectionCommand(),
            post_operation = lambda cmd: self.scan_for_btn_pose(),
            description = "request detection"
        )
        self.addCommand(        # TODO: maybe disable collisions for this object
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(self.btn_pose),
                                         cmd.setShape([0.2, 0.1, 0.0001]),
                                         cmd.setName("btn")),
            description="add button to world"
        )
        self.addCommand(
            PoseCommand(self.executor),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(),
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
        
        # TODO: maybe remove the objects added to the world
