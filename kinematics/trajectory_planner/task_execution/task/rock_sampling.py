from .task import *
from kinematics_utils.pose_corrector_new import FingersList


class RockPicking(Task):
    def __init__(self, executor):
        self.approach_distance_steps = [0.3, 0.1]
        super().__init__(executor)
        self.grip_distance = 0.03       # distance of gripper that grips rock
        self.above_distance = 0.25
        self.before_grab_distance = 0.1
    
    def aboveRockPose(self) -> qan.Pose:
        rd = pt.perception_tracker.rock_detection
        camera_pos = pc.CAMERA_TRANSFORM.position
        p = [camera_pos.x, camera_pos.y, -self.above_distance]
        pose = rd.rock_pose.point_image(p)
        return pose
    
    def grabPose(self) -> qan.Pose:
        rd = pt.perception_tracker.rock_detection
        finger_len = FingersList.DEFAULT.transform_to_eef.position.z
        rat_len = 0.02
        p = [0, 0, -rd.height/2 + finger_len - rat_len]
        return rd.rock_pose.point_image(p)
    
    def beforeGrabPose(self) -> qan.Pose:
        return self.grabPose().point_image([0, 0, self.before_grab_distance])
    
    def constructCommandChain(self):
        super().constructCommandChain()

        self.addCommand(
            RequestRockDetectionCommand(),
            description = "request new rock detection"
        )
        
        rd = pt.perception_tracker.rock_detection
        [rd.min_diameter, rd.max_diameter, rd.height]
        self.addCommand(        # TODO: maybe disable collisions for this object
            AddObjectCommand(),
            pre_operation = lambda cmd: (cmd.setPose(rd.rock_pose),
                                        cmd.setShape([rd.min_diameter, rd.max_diameter, rd.height]),
                                        cmd.setName("rock")),
            description=f"add rock to world"
        )
        
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(self.aboveRockPose()),
            description = "go above rock"
        )
        
        self.constructOpenGripperCommands()
        
        self.addCommand(
            RequestRockDetectionCommand(),
            description = "request new rock detection"
        )
        
        self.addCommand(
            PoseCommand(),
            pre_operation = lambda cmd: cmd.setPose(self.beforeGrabPose()),
            description = "go just above rock"
        )
