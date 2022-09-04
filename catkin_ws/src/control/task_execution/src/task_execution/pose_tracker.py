from geometry_msgs.msg import Pose
import task_execution.quaternion_arithmetic as qa
import rospy
import math
import copy


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
    #global DETECTED_OBJECTS_POSE
    DETECTED_OBJECTS_LOCKED = True
    DETECTION_UPDATED = True
    for _ in range(len(DETECTED_OBJECTS_POSE)):
        DETECTED_OBJECTS_POSE.pop()
    mm_to_m = 1/1000.0  # mm to m conversion
    for obj in msg.detected_objects:
        pose = DetectedObject()
        pose.id = obj.id
        pose.artag_pose.position.x = -obj.y_pos_tag*mm_to_m
        pose.artag_pose.position.y = -obj.x_pos_tag*mm_to_m
        pose.artag_pose.position.z = obj.z_pos_tag*mm_to_m
        pose.artag_pose.orientation.w = obj.x_quaternion
        pose.artag_pose.orientation.x = obj.z_quaternion
        pose.artag_pose.orientation.y = -obj.w_quaternion
        pose.artag_pose.orientation.z = obj.y_quaternion
        pose.object_pose.position.x = -obj.y_pos*mm_to_m
        pose.object_pose.position.y = -obj.x_pos*mm_to_m
        pose.object_pose.position.z = obj.z_pos*mm_to_m

        dx = -(pose.object_pose.position.x - pose.artag_pose.position.x)
        dy = pose.object_pose.position.y - pose.artag_pose.position.y
        dz = pose.object_pose.position.z - pose.artag_pose.position.z
        position = pose.artag_pose.position
        q = qa.mul(dx, qa.point_image([1,0,0], pose.artag_pose.orientation))
        position = qa.add(position, q)
        q = qa.mul(dy, qa.point_image([0,1,0], pose.artag_pose.orientation))
        position = qa.add(position, q)
        q = qa.mul(dz, qa.point_image([0,0,1], pose.artag_pose.orientation))
        position = qa.add(position, q)
        pose.object_pose.position = qa.make_point(position)

        pose.artag_pose.orientation = qa.turn_around(pose.artag_pose.orientation, [0,0,1])
        pose.object_pose.orientation = copy.deepcopy(pose.artag_pose.orientation)
        
        DETECTED_OBJECTS_POSE.append(pose)

    DETECTED_OBJECTS_LOCKED = False
    #if len(DETECTED_OBJECTS_POSE) > 0:
    #    rospy.logerr(DETECTED_OBJECTS_POSE[0].id)


def deprecate_detection():
    global DETECTION_UPDATED
    DETECTION_UPDATED = False
