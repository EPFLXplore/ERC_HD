import trajectory_planner.quaternion_arithmetic as qa
from geometry_msgs.msg import Pose
import trajectory_planner.pose_tracker as pt


def link6_transform():
    eef_correction = Pose()
    eef_correction.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    d = 0.1598
    eef_correction.position = qa.point_image([0.0, 0.0, d], eef_correction.orientation)
    return eef_correction


def finger1_transform():    # actually joint name is finger1 but the link is hd_finger2_1, will be corrected
    eef_correction = Pose()
    eef_correction.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    vect = [-0.8, 0.1005, -0.0785]
    eef_correction.position = qa.point_image(vect, eef_correction.orientation)
    return eef_correction


def init_correction(eef):
    transforms = {"link6": link6_transform, "finger1": finger1_transform}
    if eef not in transforms:
        raise ValueError(f"No end effector transform for {eef}")
    return transforms[eef]()


EEFCORRECTION = init_correction("finger1")


def correct_eef_pose(pose=None):
    if pose is None:
        pose = pt.END_EFFECTOR_POSE
    return qa.compose_poses(pose, EEFCORRECTION)


def revert(pose):
    return qa.compose_poses(pose, qa.reverse_pose(EEFCORRECTION))
