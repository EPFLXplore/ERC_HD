from geometry_msgs.msg import Pose
from custom_msg.msg import TargetInstruction
from std_msgs.msg import UInt32
import kinematics_utils.quaternion_arithmetic as qa
import math
import copy
import kinematics_utils.pose_corrector as pc


END_EFFECTOR_POSE = Pose()
DETECTED_OBJECTS_POSE = []
DETECTED_OBJECTS_LOCKED = False
DETECTION_UPDATED = False
DEPTH = 0       # [m]


class DetectedObject:
    def __init__(self, id=0):
        self.id = id
        self.object_pose = Pose()
        self.artag_pose = Pose()
    
    def str(self):
        pass
        #return str(self.pose)


def eef_pose_callback(msg):
    """listens to /arm_control/end_effector_pose topic and updates the end effector pose"""
    global END_EFFECTOR_POSE
    END_EFFECTOR_POSE = msg


def depth_callback(msg: UInt32):
    global DEPTH
    mm_to_m = 0.001
    DEPTH = msg.data * mm_to_m


def detected_object_pose_callback(msg: TargetInstruction):
    """listens to detected_elements topic and updates the pose of the detected elements (with respect to the end effector pose)"""
    global DETECTED_OBJECTS_LOCKED
    global DETECTION_UPDATED

    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True

    for _ in range(len(DETECTED_OBJECTS_POSE)):
        DETECTED_OBJECTS_POSE.pop()

    def correct_pose(pose):
        mm_to_m = 1/1000
        corrected_pose = Pose()
        corrected_pose.position.x = pose.position.x * mm_to_m
        corrected_pose.position.y = pose.position.y * mm_to_m
        corrected_pose.position.z = pose.position.z * mm_to_m
        corrected_pose.orientation = pose.orientation
        corrected_pose = pc.correct_vision_pose(corrected_pose)
        #corrected_pose = qa.compose_poses(pc.correct_eef_pose(END_EFFECTOR_POSE), corrected_pose)
        corrected_pose = qa.compose_multiple_poses(pc.correct_eef_pose(END_EFFECTOR_POSE), pc.CAMERA_TRANSFORM, corrected_pose)
        return corrected_pose
    
    corrected_artag_pose = correct_pose(msg.ar_tag_pose)
    corrected_object_pose = correct_pose(msg.object_pose)
    
    detected_object = DetectedObject()
    detected_object.artag_pose = corrected_artag_pose
    detected_object.object_pose = corrected_object_pose
    
    DETECTED_OBJECTS_POSE.append(detected_object)

    DETECTED_OBJECTS_LOCKED = False


def deprecate_detection():
    global DETECTION_UPDATED
    DETECTION_UPDATED = False
