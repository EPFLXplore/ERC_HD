from task_execution.command.command import *


class StraightMoveCommand(Command):
    """moves the end effector in a straight line by a certain distance in a certain direction without affecting its orientation"""

    def __init__(self, executor=None, axis=(1.0, 0.0, 0.0), distance=0.0, velocity_scaling_factor=1.0):
        super().__init__(executor)
        self.axis = axis
        self.distance = distance
        self.velocity_scaling_factor = velocity_scaling_factor
        self.createSetters("axis", "distance")
    
    def setAxisFromOrientation(self, orientation: Quaternion, axis: Point = (0.0, 0.0, 1.0), reverse=False):
        frame_axis = qa.make_point(axis)
        self.axis = qa.point_image(frame_axis, orientation)
        if reverse:
            self.axis = qa.mul(-1, self.axis)

    def constructPose(self):
        self.pose = Pose()
        self.pose.orientation = copy.deepcopy(pt.END_EFFECTOR_POSE.orientation)
        p = qa.list_to_point(qa.normalize(self.axis))
        p = qa.mul(self.distance, p)
        self.pose.position = qa.quat_to_point(qa.add(pt.END_EFFECTOR_POSE.position, p))

    def sesVelocityScalingFactor(self, factor):
        self.velocity_scaling_factor = factor

    def execute(self):
        """publishes on /arm_control/pose_goal topic for the trajectory planner"""
        self.constructPose()
        super().execute()
        self.executor.sendPoseGoal(self.pose, True, self.velocity_scaling_factor)
        self.finished = self.executor.waitForFeedback()
