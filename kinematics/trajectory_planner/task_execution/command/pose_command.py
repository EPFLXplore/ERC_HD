from task_execution.command.command import *
import kinematics_utils.quaternion_arithmetic as qan


class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, pose: qan.Pose = None, cartesian: bool = False, velocity_scaling_factor: float = 1.0, in_urdf_eef_frame: bool = False):
        super().__init__()
        if pose is None:
            pose = qan.Pose()
        self.setPose(pose, in_urdf_eef_frame)
        self.cartesian = cartesian
        self.velocity_scaling_factor = velocity_scaling_factor
        #self.createSetters("pose", "cartesian")
        self.retry_count = 0
        self.finished = False

    def setPose(self, pose:  qan.Pose, in_urdf_eef_frame: bool = False):
        self.pose = pose if in_urdf_eef_frame else pc.revert_to_urdf_eef(self.pose)

    def sesVelocityScalingFactor(self, factor: float):
        self.velocity_scaling_factor = factor

    def done(self) -> bool:
        return self.finished

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, self.cartesian, self.velocity_scaling_factor)
        success = self.executor.waitForFeedback()
        if success:
            self.finished = True
        elif self.retry_count < 1:
            self.has_failed = True
            self.fail_message = "couldn't execute pose command"
        self.retry_count -= 1
