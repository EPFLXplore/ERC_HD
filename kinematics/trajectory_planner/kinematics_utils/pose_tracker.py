from geometry_msgs.msg import Pose
from custom_msg.msg import ArucoObject, Rock, Brick, Ethernet, Probe, RockArray
from std_msgs.msg import UInt32
import kinematics_utils.quaternion_arithmetic_new as qan
from typing import Any, Union
import math
import copy
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc


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


def detected_object_pose_callback(msg: ArucoObject):
    """listens to detected_elements topic and updates the pose of the detected elements (with respect to the end effector pose)"""
    global DETECTED_OBJECTS_LOCKED
    global DETECTION_UPDATED

    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True

    for _ in range(len(DETECTED_OBJECTS_POSE)):
        DETECTED_OBJECTS_POSE.pop()

    def correct_pose(pose: Pose) -> Pose:
        mm_to_m = 1/1000
        corrected_pose = Pose()
        corrected_pose.position.x = pose.position.x * mm_to_m
        corrected_pose.position.y = pose.position.y * mm_to_m
        corrected_pose.position.z = pose.position.z * mm_to_m
        corrected_pose.orientation = pose.orientation
        corrected_pose = pc.vision_to_abs(corrected_pose)
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


def mm2m(pose: Pose) -> qan.Pose:
    conversion = 0.001
    pose = qan.Pose.make(pose)
    pose.position *= conversion
    return pose


MessageType = Union[ArucoObject, Rock, Brick, Ethernet, Probe]


class Detection:
    def __init__(self):
        self.locked = False
        self.deprecated = True
    
    def lock(self):
        self.locked = True
    
    def unlock(self):
        self.locked = False
    
    def deprecate(self):
        self.deprecated = True
    
    def is_detection_valid(self, msg: MessageType) -> bool:
        return msg.is_detected
        raise NotImplementedError()
    
    def callback(self, msg: MessageType):
        if not self.is_detection_valid(msg):
            return
        self.lock()
        self._callback()
        self.unlock()
        self.deprecated = False

    def _callback(self, msg: MessageType):
        raise NotImplementedError()


class FromArucoTagDetection(Detection):
    def __init__(self):
        self.artag_pose = qan.Pose()
        self.object_pose = qan.Pose()
    
    def _callback(self, msg: ArucoObject):
        def correct_pose(pose: Pose) -> Pose:
            return pc.vision_to_abs(mm2m(pose))

        self.artag_pose = correct_pose(msg.ar_tag_pose)
        self.object_pose = correct_pose(msg.object_pose)


class ButtonDetection(FromArucoTagDetection):
    @property
    def button_pose(self) -> qan.Pose:
        return self.object_pose


class RockDetection(Detection):
    def __init__(self):
        self.rock_pose = qan.Pose()
        self.max_diameter = 0.0
        self.min_diameter = 0.0
        self.height = 0.0

    def _callback(self, msg: RockArray):
        rock = msg.rocks[msg.target_index]
        vision_pose = qan.Pose()
        mm_to_m = 1/1000
        cm_to_m = 1/100
        deg_to_rad = math.pi/180
        vision_pose.position = qan.Point.make(rock.center)
        vision_pose.orientation = qan.Quaternion.from_axis_angle([0, 0, 1], rock.angle*deg_to_rad + math.pi/2)
        vision_pose = mm2m(vision_pose)
        self.rock_pose = pc.vision_to_abs(vision_pose)
        self.max_diameter = rock.max_diameter
        self.min_diameter = rock.min_diameter
        self.height = rock.height


class ProbeDetection(Detection):
    def __init__(self):
        self.pose = qan.Pose()
    
    def _callback(self, msg: Probe):
        self.pose = msg.pose


class PerceptionTracker:
    def __init__(self):
        # self.button_detection = ButtonDetection()
        self.aruco_object_detection = FromArucoTagDetection()
        self.rock_detection = RockDetection()
        self.probe_detection = ProbeDetection()


perception_tracker = PerceptionTracker()
