from geometry_msgs.msg import Pose
import task_execution.quaternion_arithmetic as qa


END_EFFECTOR_POSE = Pose()
DETECTED_OBJECTS_POSE = []
DETECTED_OBJECTS_LOCKED = False
DETECTION_UPDATED = False


def eef_pose_callback(msg):
    """listens to /arm_control/end_effector_pose topic and updates the end effector pose"""
    global END_EFFECTOR_POSE
    END_EFFECTOR_POSE = msg


def detected_objects_pose_callback(msg):
    """listens to detected_elements topic and updates the pose of the detected elements (with respect to the end effector pose)"""
    global DETECTED_OBJECTS_LOCKED
    global DETECTION_UPDATED
    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True
    DETECTED_OBJECTS_POSE.clear()
    mm_to_m = 1/1000  # mm to m conversion
    for obj in msg.detected_objects:
        pose = Pose()
        pose.position.x = -obj.y_pos*mm_to_m
        pose.position.y = -obj.x_pos*mm_to_m
        pose.position.z = obj.z_pos*mm_to_m
        pose.orientation.w = obj.w_quaternion
        pose.orientation.x = -obj.y_quaternion
        pose.orientation.y = obj.z_quaternion
        pose.orientation.z = obj.x_quaternion
        DETECTED_OBJECTS_POSE.append(pose)
    DETECTED_OBJECTS_LOCKED = False


def deprecate_detection():
    global DETECTION_UPDATED
    DETECTION_UPDATED = False
