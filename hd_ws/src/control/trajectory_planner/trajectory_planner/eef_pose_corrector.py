import trajectory_planner.quaternion_arithmetic as qa
from geometry_msgs.msg import Pose
import trajectory_planner.pose_tracker as pt


def init_correction():
    eef_correction = Pose()
    eef_correction.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=2.975)
    d = 0.1598
    eef_correction.position = qa.point_image([0.0, 0.0, d], eef_correction.orientation)
    return eef_correction


EEFCORRECTION = init_correction()


def correct_eef_pose(pose=None):
    if pose is None:
        pose = pt.END_EFFECTOR_POSE
    return qa.compose_poses(pose, EEFCORRECTION)


def revert(pose):
    return qa.compose_poses(pose, qa.reverse_pose(EEFCORRECTION))
