from __future__ import annotations
from .task import *
from kinematics_utils.pose_corrector_new import Tool, ToolsList
from custom_msg.msg import HDGoal


class ToolManip(Task):
    TOOL_MAINTAINING_TORQUE_ID = "tool_maintaining_torque"
    def __init__(self, executor: Executor, tool: str, declare_tool_maintaining_torque_cmd: bool = True):
        tool = ToolsList.FROM_ROS_MSG_MAP[tool]
        super().__init__(executor, construct_command_chain=False)
        self.tool: Tool = tool
        self.above_distance = 0.15
        self.tool_maintaining_torque_scaling = 0.1
        if declare_tool_maintaining_torque_cmd:
            self.declareBackgroundCommand(
                id = self.TOOL_MAINTAINING_TORQUE_ID,
                command = GripperBackgroundCommand(action=GripperBackgroundCommand.CLOSE, torque_scaling_factor=self.tool_maintaining_torque_scaling),
                description = "Starting tool maintaining torque"
            )
        self.constructCommandChain()
    
    def aboveToolPose(self) -> qan.Pose:
        d = 0.001 + self.above_distance
        p = [0.0, 0.0, -d]
        tool_pose = self.tool.pickup_pose
        tool_pose.position = tool_pose.point_image(p)
        return tool_pose

    def equipToolCommand(self):
        class EquipToolCommand(Command):
            def execute(s):
                super().execute()
                pc.equip_tool(self.tool)
        self.addCommand(
            EquipToolCommand(),
            description = "equip tool"
        )

    def unequipToolCommand(self):
        class UnequipToolCommand(Command):
            def execute(s):
                super().execute()
                pc.unequip_tool(self.tool)
        self.addCommand(
            UnequipToolCommand(),
            description = "unequip tool"
        )


class ToolPickup(ToolManip):
    def constructCommandChain(self):
        super().constructCommandChain()
        
        self.addCommand(
            PoseCommand(pose=self.aboveToolPose(), in_urdf_eef_frame=True),
            description = "go above tool"
        )
        
        self.constructOpenGripperCommands()

        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2, distance=self.above_distance),
            pre_operation = lambda cmd: cmd.setAxisFromOrientation(pc.correct_eef_pose().orientation),
            description = "Reach tool"
        )

        self.constructCloseGripperCommands()
        
        self.setBackgroundCommandStartPoint(
            id = self.TOOL_MAINTAINING_TORQUE_ID,
        )

        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2, distance=self.above_distance),
            pre_operation = lambda cmd: cmd.setAxisFromOrientation(pc.correct_eef_pose().orientation, reverse=True),
            description = "Retract with tool"
        )
        
        self.equipToolCommand()


class ToolPlaceback(ToolManip):
    def constructCommandChain(self):
        super().constructCommandChain()
        
        self.addCommand(
            PoseCommand(pose=self.aboveToolPose(), in_urdf_eef_frame=True),
            description = "go above tool station"
        )
        
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2, distance=self.above_distance),
            pre_operation = lambda cmd: cmd.setAxisFromOrientation(pc.correct_eef_pose().orientation),
            description = "Reach tool station"
        )

        self.setBackgroundCommandStopPoint(
            id = self.TOOL_MAINTAINING_TORQUE_ID,
            # description = "stopping background gripper torque",
            # allow_lazy_cmd_retrieval = True
        )
        
        self.constructOpenGripperCommands()
        
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2, distance=self.above_distance),
            pre_operation = lambda cmd: cmd.setAxisFromOrientation(pc.correct_eef_pose().orientation, reverse=True),
            description = "Retract away from tool station"
        )

        self.unequipToolCommand()
