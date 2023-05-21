from task_execution.command.command import *


class PoseCommand(Command):
    """moves the arm to a requested pose (position + orientation of the end effector)"""

    def __init__(self, executor=None, pose=None, cartesian=False):
        super().__init__(executor)
        self.pose = pose    # Pose
        self.cartesian = cartesian
        self.createSetters("pose", "cartesian")
        self.finished = False

    def setPose(self, position=None, orientation=None):#, revert=True):
        if self.pose is None:
            self.pose = Pose()
        if position is not None: 
            self.pose.position = position
        if orientation is not None: 
            self.pose.orientation = orientation
        
        self.pose = pc.global_revert(self.pose)

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, self.cartesian)
        self.finished = self.executor.waitForFeedback()
