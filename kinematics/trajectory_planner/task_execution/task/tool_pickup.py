from __future__ import annotations
from .task import *
from kinematics_utils.pose_corrector_new import Tools


TOOL_MAINTAINING_TORQUE_ID = "tool_maintaining_torque"


class ToolPickup(Task):
    def __init__(self, tool: int):
        if tool not in Tools.SWAPABLE:
            raise ValueError("Tool is unknown or isn't swapable")
        super().__init__()
        self.tool = tool
        self.above_distance = 0.2
        self.tool_maintaining_torque_scaling = 0.8
        self.declareBackgroundCommand(
            id = TOOL_MAINTAINING_TORQUE_ID,
            command = GripperBackgroundCommand(action=GripperBackgroundCommand.CLOSE, torque_scaling_factor=self.tool_maintaining_torque_scaling),
            description = "Starting tool maintaining torque"
        )
    
    def aboveToolPose(self) -> qan.Pose:
        d = 0.001 + self.above_distance
        p = [0.0, 0.0, d]
        tool_pose = Tools.PICKUP_POSE[self.tool]
        return tool_pose.point_image(p)
    
    def constructCommandChain(self):
        super().constructCommandChain()
        
        self.addCommand(
            PoseCommand(pose=self.aboveToolPose()),
            description = "go above tool"
        )
        
        self.addCommand(
            GripperCommand(action=GripperCommand.OPEN, duration=3.0, torque_scaling_factor=0.8),
            description = "Open gripper"
        )
        
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.5, distance=self.above_distance),
            pre_operation = lambda cmd: cmd.setAxisFromOrientation(pc.correct_eef_pose().orientation),
            description = "Reach tool"
        )

        self.setBackgroundCommandStartPoint(
            id = TOOL_MAINTAINING_TORQUE_ID,
        )
        
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


