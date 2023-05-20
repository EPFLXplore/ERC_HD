from geometry_msgs.msg import Pose
import trajectory_planner.quaternion_arithmetic as qa
import math
import copy
from interfaces.msg import PanelObject
import trajectory_planner.eef_pose_corrector as epc


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


def detected_object_pose_callback(msg: PanelObject):
    """listens to detected_elements topic and updates the pose of the detected elements (with respect to the end effector pose)"""
    global DETECTED_OBJECTS_LOCKED
    global DETECTION_UPDATED

    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True

    for _ in range(len(DETECTED_OBJECTS_POSE)):
        DETECTED_OBJECTS_POSE.pop()

    #mm_to_m = 1/1000.0  # mm to m conversion

    cm_to_m = 1/100

    corrected_pose = Pose()
    corrected_pose.position.x = msg.pose.position.x * cm_to_m
    corrected_pose.position.y = msg.pose.position.y * cm_to_m
    corrected_pose.position.z = msg.pose.position.z * cm_to_m
    corrected_pose.orientation = msg.pose.orientation

    corected_pose = epc.correct_vision_pose(corrected_pose)

    detected_object = DetectedObject()
    detected_object.artag_pose = corected_pose
    detected_object.object_pose = corected_pose
    DETECTED_OBJECTS_POSE.append(detected_object)

    DETECTED_OBJECTS_LOCKED = False


def deprecate_detection():
    global DETECTION_UPDATED
    DETECTION_UPDATED = False
