from geometry_msgs.msg import Pose
import task_execution.quaternion_arithmetic as qa
import rospy


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
        return str(self.pose)


def eef_pose_callback(msg):
    """listens to /arm_control/end_effector_pose topic and updates the end effector pose"""
    global END_EFFECTOR_POSE
    END_EFFECTOR_POSE = msg


def detected_objects_pose_callback(msg):
    """listens to detected_elements topic and updates the pose of the detected elements (with respect to the end effector pose)"""
    global DETECTED_OBJECTS_LOCKED
    global DETECTION_UPDATED
    global DETECTED_OBJECTS_POSE
    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True
    DETECTED_OBJECTS_POSE = []
    mm_to_m = 1/1000.0  # mm to m conversion
    for obj in msg.detected_objects:
        pose = DetectedObject()
        pose.id = obj.id
        pose.object_pose.position.x = -obj.y_pos*mm_to_m
        pose.object_pose.position.y = -obj.x_pos*mm_to_m
        pose.object_pose.position.z = obj.z_pos*mm_to_m
        pose.object_pose.orientation.w = obj.w_quaternion
        pose.object_pose.orientation.x = -obj.y_quaternion
        pose.object_pose.orientation.y = obj.z_quaternion
        pose.object_pose.orientation.z = obj.x_quaternion
        pose.artag_pose.position.x = -obj.y_pos_tag*mm_to_m
        pose.artag_pose.position.y = -obj.x_pos_tag*mm_to_m
        pose.artag_pose.position.z = obj.z_pos_tag*mm_to_m
        pose.artag_pose.orientation.w = obj.w_quaternion
        pose.artag_pose.orientation.x = -obj.y_quaternion
        pose.artag_pose.orientation.y = obj.z_quaternion
        pose.artag_pose.orientation.z = obj.x_quaternion
        DETECTED_OBJECTS_POSE.append(pose)
    DETECTED_OBJECTS_LOCKED = False
    #if len(DETECTED_OBJECTS_POSE) > 0:
    #    rospy.logwarn(DETECTED_OBJECTS_POSE[0].str())


def deprecate_detection():
    global DETECTION_UPDATED
    DETECTION_UPDATED = False
