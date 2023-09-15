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

        for target in self.approach_distance_steps:
            self.addCommand(
                RequestDetectionCommand(),
                post_operation = lambda cmd: self.scanForObjects(),
                description = "request detection"
            )
            self.addCommand(
                AddObjectCommand(),
                pre_operation = lambda cmd: (cmd.setPose(self.object_pose),
                                            cmd.setShape([0.15, 0.1, 0.05]),
                                            cmd.setName("rock")),
                description="add rock to world"
            )
            self.addCommand(
                PoseCommand(),
                pre_operation = lambda cmd: cmd.setPose(position=self.getApproachPosition(target),
                                                        orientation=self.getApproachOrientation()),
                description = f"approach rock at {target} meters"
            )

        self.constructOpenGripperCommands()

        self.addCommand(
            AddObjectCommand(name="rock", operation=Object.REMOVE),
            description="remove rock from collisions"
        )

        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2),
            pre_operation = lambda cmd: (cmd.setDistance(self.approach_distance_steps[-1] + self.grip_distance),
                                         cmd.setAxisFromOrientation(self.object_pose.orientation, reverse=True)),
            description="advance to rock"
        )

        self.constructCloseGripperCommands(high_torque=1.0, low_torque=0.5)


class RockSamplingDrop(Task):
    def __init__(self, executor):
        super().__init__(executor)
        self.rise_distance = 0.3    # after picking the rock, before going to a default position

    def constructCommandChain(self):
        super().constructCommandChain()

        self.constructDropCommands()

    def constructDropCommands(self):
        # go up some using cartesian path
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.7),
            pre_operation = lambda cmd: (cmd.setDistance(self.rise_distance),
                                         cmd.setAxisFromOrientation(pc.correct_eef_pose().orientation, reverse=True)),
            description = "rise up"
        )

        self.addCommand(
            NamedJointTargetCommand(name="optimal_view"),
            description = "go to optimal view position"
        )

        # go above science container
        self.addCommand(
            NamedJointTargetCommand(name="above_science"),
            description = "go above science container"
        )

        # go above science container
        self.addCommand(
            NamedJointTargetCommand(name="science_rock_drop"),
            description = "go to drop position"
        )

        self.constructOpenGripperCommands(post_completion_wait=1.0)

        self.constructCloseGripperCommands()

        self.addCommand(
            NamedJointTargetCommand(name="above_science"),
            description = "go back above science"
        )

        self.addCommand(
            NamedJointTargetCommand(name="optimal_view"),
            description = "go back to optimal view position"
        )


class RockSamplingComplete(RockSamplingApproach, RockSamplingDrop):
    def __init__(self, executor):
        RockSamplingApproach.__init__(self, executor)
        RockSamplingDrop.__init__(self, executor)
    
    def constructCommandChain(self):
        RockSamplingApproach.constructCommandChain(self)
        RockSamplingDrop.constructCommandChain(self)
