from task_execution.command.command import *


class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, executor=None, pose=None, cartesian=False, velocity_scaling_factor=1.0):
        super().__init__(executor)
        self.pose = pose    # Pose
        self.cartesian = cartesian
        self.velocity_scaling_factor = velocity_scaling_factor
        #self.createSetters("pose", "cartesian")
        self.retry_count = 0
        self.finished = False

    def setPose(self, position=None, orientation=None):
        if self.pose is None:
            self.pose = Pose()
        if position is not None: 
            self.pose.position = position
        if orientation is not None: 
            self.pose.orientation = orientation
 
        self.pose = pc.global_revert(self.pose)

    def sesVelocityScalingFactor(self, factor):
        self.velocity_scaling_factor = factor

    def done(self):
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
        self.retry_count -= 1
