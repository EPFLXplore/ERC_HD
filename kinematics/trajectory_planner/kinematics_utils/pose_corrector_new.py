from typing import Optional
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray
from math import pi


# TRANSFORM CONSTRUCTORS

def link6_transform():
    # !!!=== deprecated ===!!!
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    d = 0.1598
    transform.position = qa.point_image([0.0, 0.0, d], transform.orientation)
    return transform


def finger1_transform_old():    # actually joint name is finger1 but the link is hd_finger2_1, will be corrected
    # !!!=== deprecated ===!!!
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    vect = [-0.8, 0.1005, -0.0785]
    transform.position = qa.point_image(vect, transform.orientation)
    return transform


def finger1_transform():
    # !!!=== deprecated ===!!!
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2)
    vect = [-0.02, -0.743, -0.419]       # [-0.097, -0.7545, -0.3081]
    transform.position = qa.point_image(vect, transform.orientation)
    return transform


def new_finger_transform():
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2)
    vect = [0.0, 0.0, 0.2018 + 0.06 + 0.0037]
    transform.position = qa.point_image(vect, transform.orientation)
    return transform


def gripper_transform() -> qan.Pose:
    transform = qan.Pose()
    transform.orientation = qan.Quaternion.from_axis_angle(axis=(1.0, 0.0, 0.0), angle=pi/2)    # * qan.Quaternion.from_axis_angle(axis=(0.0, 0.0, 1.0), angle=pi)
    offset = qan.Point(z=0.0916)
    transform.position = transform.orientation.point_image(offset)
    return transform


def construct_eef_transform(eef: str) -> qan.Pose:
    transforms = {"link6": link6_transform, "finger1": finger1_transform, "new_finger": new_finger_transform, "gripper": gripper_transform}
    if eef not in transforms:
        raise ValueError(f"No end effector transform for {eef}")
    return qan.Pose.make(transforms[eef]())


class Tools:
    """
    Poses of eef when equiped with different tools, transforms with respect to gripper without tools
    """
    NONE = 0
    FINGERS = 1
    BUTTONS = 2
    PROBES = 3
    SHOVEL = 4
    VOLTMETER = 5
    NON_SWAPABLE = {FINGERS, PROBES}
    SWAPABLE = {BUTTONS, SHOVEL, VOLTMETER}
    LIST = {NONE} | NON_SWAPABLE | SWAPABLE
    PICKUP_POSE = {
        BUTTONS: qan.Pose(),
        PROBES: qan.Pose()
    }
    POSE = {
        NONE: qan.Pose(),
        FINGERS: qan.Pose(position=qan.Point(z=0.0867)),
        BUTTONS: qan.Pose(position=qan.Point(z=0.136)),
        PROBES: qan.Pose(position=qan.Point(z=0.039))
    }


class PoseCorrector:
    GRIPPER_TRANSFORM_CORRECTION = construct_eef_transform("gripper")
    CAMERA_TRANSFORM = qan.Pose(                    # transform between end effector and camera
        position = qan.Point(x=-0.019, y=-0.07, z=-0.051 - 0.0037)    # 0.051 to camera glass and 0.0037 to focal point    # old x=-0.009 y=-0.063
    )
    
    def __init__(self, tool: int = Tools.BUTTONS):
        self.tool = tool
    
    def set_tool(self, tool: int):
        if tool not in Tools.LIST:
            raise ValueError("Unknown tool")
        self.tool = tool
    
    def set_camera_transform(self, pose: Pose):
        self.CAMERA_TRANSFORM = qan.Pose.make(pose)
    
    def set_camera_transform_position(self, position: Float64MultiArray):
        data = position.data
        self.CAMERA_TRANSFORM.position = qan.Point(x=data[0], y=data[1], z=data[2])
    
    def tool_transform(self) -> qan.Pose:
        return Tools.POSE[self.tool]
    
    def correct_gripper_pose(self, pose: Optional[Pose] = None) -> qan.Pose:
        """
        pose: urdf end of chain pose in abs frame
        """
        if pose is None:
            import kinematics_utils.pose_tracker as pt
            pose = pt.END_EFFECTOR_POSE
        return pose @ self.GRIPPER_TRANSFORM_CORRECTION
    
    def correct_eef_pose(self, pose: Optional[Pose] = None) -> qan.Pose:
        """
        pose: urdf end of chain pose in abs frame
        """
        return self.correct_gripper_pose(pose) @ self.tool_transform()

    def revert_to_urdf_eef(self, pose: Pose) -> qan.Pose:
        """
        pose: in eef frame
        """
        return pose @ self.tool_transform().inv() @ self.GRIPPER_TRANSFORM_CORRECTION.inv()
        return pose @ self.correct_eef_pose().inv()

    def abs_to_eef(self, pose: Pose) -> qan.Pose:
        # input: pose in absolute frame
        # output: pose in eef frame
        return self.correct_eef_pose().inv() @ pose
    
    def eef_to_abs(self, pose: Pose) -> qan.Pose:
        # input: pose in eef frame
        # output: pose in absolute frame
        return self.correct_eef_pose() @ pose

    def vision_to_abs(self, pose: Pose) -> qan.Pose:
        # input: pose in the frame of the camera (vision frame)
        # output: pose in absolute frame
        return self.correct_gripper_pose() @ self.CAMERA_TRANSFORM @ pose

    def abs_to_vision(self, pose: Pose) -> qan.Pose:
        # input: pose in absolute frame
        # output: pose in camera frame (vision frame)
        return self.CAMERA_TRANSFORM.inv() @ self.correct_gripper_pose().inv() @ pose

    def correct_vision_pose(self, pose: Pose) -> qan.Pose:
        # !!!=== deprecated ===!!!
        return qan.Pose.make(pose)

    def revert_from_vision(self, pose: Pose) -> qan.Pose:
        # !!!=== deprecated ===!!!
        return qan.Pose.make(pose)

    def revert_to_vision(pose: Pose) -> qan.Pose:     # TODO: think about these functions (pre or post composing with the tranform) + give better names
        # only for simulating vision and its reference
        # TODO: wtf is this function
        # !!!=== deprecated ===!!!
        return qan.Pose.make(pose)


POSE_CORRECTOR = PoseCorrector()
