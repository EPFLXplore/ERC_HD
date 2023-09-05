from geometry_msgs.msg import Pose
import kinematics_utils.quaternion_arithmetic as qa
import math
import copy
import kinematics_utils.pose_corrector as pc


END_EFFECTOR_POSE = Pose()
DETECTED_OBJECTS_POSE = []
DETECTED_OBJECTS_LOCKED = False
DETECTION_UPDATED = False


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


def detected_object_pose_callback(msg: Pose):
    """listens to detected_elements topic and updates the pose of the detected elements (with respect to the end effector pose)"""
    global DETECTED_OBJECTS_LOCKED
    global DETECTION_UPDATED

    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True

    for _ in range(len(DETECTED_OBJECTS_POSE)):
        DETECTED_OBJECTS_POSE.pop()

    mm_to_m = 1/1000

    corrected_pose = Pose()
    corrected_pose.position.x = msg.position.x * mm_to_m
    corrected_pose.position.y = msg.position.y * mm_to_m
    corrected_pose.position.z = msg.position.z * mm_to_m
    corrected_pose.orientation = msg.orientation

    corrected_pose = pc.correct_vision_pose(corrected_pose)

    #corrected_pose = qa.compose_poses(pc.correct_eef_pose(END_EFFECTOR_POSE), corrected_pose)
    corrected_pose = qa.compose_multiple_poses(pc.correct_eef_pose(END_EFFECTOR_POSE), pc.CAMERA_TRANSFORM, corrected_pose)
    
    detected_object = DetectedObject()
    detected_object.artag_pose = corrected_pose
    detected_object.object_pose = corrected_pose
    
    DETECTED_OBJECTS_POSE.append(detected_object)

    DETECTED_OBJECTS_LOCKED = False


def deprecate_detection():
    global DETECTION_UPDATED
    DETECTION_UPDATED = False
