from .task import *


class RockSamplingApproach(Task):
    def __init__(self, executor):
        self.approach_distance_steps = [0.3, 0.1]
        super().__init__(executor)
        self.grip_distance = 0.03       # distance of gripper that grips rock

    def getApproachPosition(self, target):
        return qa.point_add(self.object_pose.position, qa.point_image([0.0, 0.0, target], self.object_pose.orientation))
    
    def getApproachOrientation(self):
        return qa.turn_around(self.object.orientation)
    
    def constructCommandChain(self):
        super().constructCommandChain()
        # only dropping the rock, the detection and picking part is manual for now
        self.constructDropCommands()
    
    def constructApproachCommands(self):
        self.addCommand(
            NamedJointTargetCommand(name="face_ground"),
            description = "face ground"
        )

        