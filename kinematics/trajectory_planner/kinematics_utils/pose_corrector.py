import kinematics_utils.quaternion_arithmetic as qa
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray
import kinematics_utils.pose_tracker as pt
from math import pi


# TRANSFORM CONSTRUCTORS

def link6_transform():
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    d = 0.1598
    transform.position = qa.point_image([0.0, 0.0, d], transform.orientation)
    return transform


def finger1_transform_old():    # actually joint name is finger1 but the link is hd_finger2_1, will be corrected
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    vect = [-0.8, 0.1005, -0.0785]
    transform.position = qa.point_image(vect, transform.orientation)
    return transform


def finger1_transform():
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2)
    vect = [-0.02, -0.743, -0.419]       # [-0.097, -0.7545, -0.3081]
    transform.position = qa.point_image(vect, transform.orientation)
    return transform


def new_finger_transform():
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2)
    vect = [0.0, 0.0, 0.2018]
    transform.position = qa.point_image(vect, transform.orientation)
    return transform


def construct_eef_transform(eef):
    transforms = {"link6": link6_transform, "finger1": finger1_transform, "new_finger": new_finger_transform}
    if eef not in transforms:
        raise ValueError(f"No end effector transform for {eef}")
    return transforms[eef]()


def construct_vision_tranform():
    transform = Pose()
    transform.orientation = qa.quat([0.0, 0.0, 1.0], pi/2)
    return transform


# STATIC TRANSFORMS

EEF_TRANSFORM_CORRECTION = construct_eef_transform("new_finger")
EEF_TRANSFORM_CORRECTION.position = qa.point_add(EEF_TRANSFORM_CORRECTION.position, Point(x=-0.028))

CAMERA_TRANSFORM = Pose(                    # transform between end effector and camera
    position = Point(x=0.051, y=0.0, z=-0.115)    # X = 0.0447, y = -0.009
)
CAMERA_TRANSFORM.position = qa.point_add(CAMERA_TRANSFORM.position, Point(z=-0.028))

VISION_TRANSFORM_CORRECTION = Pose()
VISION_TRANSFORM_CORRECTION.orientation = qa.quat([0.0, 0.0, 1.0], pi/2)    # vision has different frame than MoveIt


# CORRECTORS

def correct_eef_pose(pose=None):
    if pose is None:
        pose = pt.END_EFFECTOR_POSE
    return qa.compose_poses(pose, EEF_TRANSFORM_CORRECTION)


def revert_to_eef(pose):
    return qa.compose_poses(pose, qa.reverse_pose(EEF_TRANSFORM_CORRECTION))


def abs_to_eef(pose):
    # input: pose in absolute from
    # output: pose in eef frame
    eef_pose = correct_eef_pose()
    return qa.compose_poses(qa.reverse_pose(eef_pose), pose)


def correct_vision_pose(pose):
    return qa.compose_poses(VISION_TRANSFORM_CORRECTION, pose)


def revert_from_vision(pose):
    return qa.compose_poses(pose, qa.reverse_pose(VISION_TRANSFORM_CORRECTION))


def revert_to_vision(pose):     # TODO: think about these functions (pre or post composing with the tranform) + give better names
    # only for simulating vision and its reference
    pose = qa.compose_poses(qa.reverse_pose(VISION_TRANSFORM_CORRECTION), pose)
    return qa.compose_poses(pose, qa.reverse_pose(VISION_TRANSFORM_CORRECTION))


def global_revert(pose):
    pose = revert_from_vision(pose)
    pose = revert_to_eef(pose)
    return pose


# setters
def set_camera_transform_position(position: Float64MultiArray):
    data = position.data
    CAMERA_TRANSFORM.position = Point(x=data[0], y=data[1], z=data[2])
