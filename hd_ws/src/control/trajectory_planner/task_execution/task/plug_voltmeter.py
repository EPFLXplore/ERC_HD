from .task import *


class PlugVoltmeter(Task):
    def __init__(self, executor, pose=None):
        super().__init__(executor)
        self.artag_pose = None
        self.scan_distance = 0.13
        self.press_distance = 0.2
        self.plug_distance = 0.015        # distance between end effector and control panel when the voltemeter is plugged
        self.pause_time = 0.5

    @property
    def plug_pose(self):
        return self.object_pose

    def getPressPosition(self):
        d = 0.001 + self.press_distance + self.plug_distance
        p = [0.0, 0.0, d]
        return qa.point_object_image(p, self.plug_pose)

    def getPressOrientation(self):
        return qa.turn_around(self.plug_pose.orientation)
    
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
        
        self.constructVoltmeterExtensionCommands()
        self.constructStandardDetectionCommands("plug", extended=True)
        self.constructApproachCommands()
        self.constructVoltmeterRetractionCommands()

        # TODO: maybe remove the objects added to the world

    def constructVoltmeterExtensionCommands(self):
        self.addCommand(
            GripperCommand(self, GripperCommand.OPEN, duration=3.0),
            description = "open gripper"
        )
        self.addCommand(
            VoltmeterCommand(self, VoltmeterCommand.EXTEND),
            description = "extend voltmeter"
        )
        self.addCommand(
            GripperCommand(self, GripperCommand.CLOSE, duration=3.0),
            description = "clamp gripper around voltmeter"
        )
    
    def constructApproachCommands(self):
        self.addCommand(
            PoseCommand(self.executor),
            pre_operation = lambda cmd: cmd.setPose(position=self.getPressPosition(),
                                                    orientation=self.getPressOrientation()),
            description = "go in front of button"
        )
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.1),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.plug_pose.orientation, reverse=True)),
            description = "click on button"
        )
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.1),
            pre_operation = lambda cmd: (cmd.setDistance(self.press_distance),
                                         cmd.setAxisFromOrientation(self.plug_pose.orientation)),
            description = "move back from button"
        )

    def constructVoltmeterRetractionCommands(self):
        self.addCommand(
            GripperCommand(self, GripperCommand.OPEN, duration=2.0),
            description = "open gripper"
        )
        self.addCommand(
            VoltmeterCommand(self, VoltmeterCommand.RETRACT),
            description = "extend voltmeter"
        )
        self.addCommand(
            GripperCommand(self, GripperCommand.CLOSE, duration=4.0),
            description = "clamp gripper around voltmeter"
        )
