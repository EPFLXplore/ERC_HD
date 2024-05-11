from .task import *
from task_execution.command import BackgroundGripperCommand
import numpy as np
from scipy.spatial.transform import Rotation
import kinematics_utils.quaternion_arithmetic_new as qan

# 1) Move to approach position (shifted along z and x of the probe)
# From vision, get goal pose in rover base referential, so need to convert it in end effector referential, and shift the z with approach distance
# Move to computed approach pose
# 2) Recompute goal position and orientation: # VISION PART 
# 3) Open gripper/special end piece?
# 4) Move to: position = goal position, orientation = goal orientation
# 5) Close gripper/special end piece?
# 6) Move to above science container
# 7) Open gripper
# 8) Move to home position


class ProbeSampling(Task):
    def __init__(self, executor):
        self.shift = 0 # length of big part of probe???
        self.approach_distance = 0.3 # distance along z between target and approach position of end effector
        self.grip_distance = 0.03   
        self.thresh = np.pi/4
        self.probe_station_id = 0

        super().__init__(executor)

    def setStationId(self, i):
        self.probe_station_id = i

    def computeQuaternion(self):
        # Compute the vectors (u,v,w) which define the probe referential
        w = self.object_pose.orientation.point_image([0.0, 0.0, 1.0])

        z_ref = qan.Point.make([0,0,1])#np.array([0,0,1])
        
        cost = z_ref.dot(w) / (w.norm()*z_ref.norm())
        sint = z_ref.cross(w) / (w.norm()*z_ref.norm())
        angle_z = qan.Quaternion.reverse_trig(cost, sint)
        # if angle btwn z_ref and w is too big, direction of approach is vertical
        if(abs(angle_z)>self.thresh): # considered the absolute value so a negative angle is also taken into account
            l = z_ref
        else:
            l = self.object_pose.position

        u = l - l.dot(w) / w.norm()* w
        v = w.cross(u)

        base_ref = np.array([[1,0,0],[0,1,0],[0,0,1]])
        probe_ref = np.array([u.xyz,v.xyz,w.xyz])

        R = Rotation.from_matrix(base_ref.T @ probe_ref) # Matrix of rotation

        n = np.array([(R[2,1]-R[1,2]),(R[0,2]-R[2,0]),(R[1,0]-R[0,1])]) # Axis of rotation 

        Kn = np.array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        cos_angle = (np.trace(R)-1)/2 # Angle of rotation
        sin_angle = -(np.trace(Kn*R)/2)
        angle = qan.Quaternion.reverse_trig(cos_angle, sin_angle)
        quaternion=qan.Quaternion.from_axis_angle(n, angle)

        return quaternion
    
    def getApproachPosition(self): 
        #return qa.point_add(self.object_pose.position, qa.point_image([self.approach_distance, 0.0, self.shift], self.computeQuaternion()))
        return self.object_pose.position + self.computeQuaternion().point_image([self.approach_distance, 0.0, self.shift]) 
    def getApproachOrientation(self):
        return self.computeQuaternion().turn_around()
    
    def constructCommandChain(self):
        super().constructCommandChain()

        # 1) Move to approach distance: shifted along x_p by approach_distance
        self.addCommand(
                PoseCommand(),
                pre_operation = lambda cmd: cmd.setPose(position=self.getApproachPosition(),
                                                        orientation=self.getApproachOrientation()),
                description = f"move to approach pose"
            )
        
        # 2) Recompute goal pose: VISION 
        ## update position_baseRef, orientation_baseRef

        # 3) Open gripper
        self.addCommand(
            GripperCommand(self, GripperCommand.OPEN, duration=2.0, torque_scaling_factor=0.1),
            description = "open gripper"
        )

        # 4) Move to goal pose, why + grip distance and not minus?
        self.addCommand(
            StraightMoveCommand(velocity_scaling_factor=0.2),
            pre_operation = lambda cmd: (cmd.setDistance(self.approach_distance + self.grip_distance), 
                                         cmd.setAxisFromOrientation(self.getApproachOrientation(), axis=[-1.0, 0.0, 0.0])),
            description="advance to probe"
        )

        # 5) Close gripper
        self.setBackgroundCommandStartPoint("close gripper")

        # 6) Move to above science container
        self.addCommand(
            NamedJointTargetCommand(name="science_drop"),
            description = "go above science container"
        )

        # 7) Open gripper
        # self.addCommand(
        #     GripperCommand(self, GripperCommand.OPEN, duration=1.0, torque_scaling_factor=1.0),
        #     description = "open gripper"
        # )
        # self.addCommand(
        #     GripperCommand(self, GripperCommand.OPEN, duration=2.0, torque_scaling_factor=0.1),
        #     description = "open gripper"
        # )

        self.declareBackgroundCommand(
            "close gripper",
            BackgroundGripperCommand(action=BackgroundGripperCommand.CLOSE, torque_scaling_factor=0.5),
            description="maintain closed gripper"
        )

        

        # 8) Move to home position
        self.addCommand(
            NamedJointTargetCommand(name="home"),
            description = "move to home position"
        )

        self.addCommand(
            NamedJointTargetCommand(name=f"probe_station{self.probe_station_id}"),
            description = "move to home position"
        ) # needs to be 

        self.setBackgroundCommandStopPoint("close gripper")





