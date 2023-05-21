from task_execution.command.command import *


class GripperRotationCommand(Command):
    """rotates the gripper around the given axis by the given angle without affecting its position"""

    def __init__(self, executor=None, axis=(0,0,1), angle=0):
        super().__init__(executor)
        self.axis = axis
        self.angle = angle
        self.createSetters("axis", "angle")
    
    def constructPose(self):
        self.pose = Pose()
        self.pose.position = copy.deepcopy(pt.END_EFFECTOR_POSE.position)
        q = qa.quat(self.axis, self.angle)
        o = qa.mul(q, pt.END_EFFECTOR_POSE.orientation)
        self.pose.orientation = qa.quat_normalize(o)
    
    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        super().execute()
        self.executor.sendPoseGoal(self.pose, True)
        self.finished = self.executor.waitForFeedback()


class GripperManipulationCommand(Command):
    """opens/closes the gripper to a desired position"""
    def __init__(self, executor):
        super().__init__(executor)

    def execute(self):
        """publishes on /arm_control/joint_cmd topic for the motor controller"""
        super().execute()
