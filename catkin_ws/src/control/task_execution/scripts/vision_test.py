#!/usr/bin/env python

import rospy
from task_execution.task_classes import *
import task_execution.quaternion_arithmetic as qa
from task_execution.msg import Task
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
import time
from math import pi
from vision_no_ros.msg import panel_object, object_list

add_box_pub = rospy.Publisher("/arm_control/world_update", Pose, queue_size=5)
remove_box_pub = rospy.Publisher("/arm_control/remove_box", Bool, queue_size=5)


#end_effector_pose = Pose()
artag_pose = Pose()


def artag_callback(msg):
    o = msg.detected_objects[0]
    artag_pose.position.x = o.x_pos/1000
    artag_pose.position.y = o.y_pos/1000
    artag_pose.position.z = o.z_pos/1000
    artag_pose.orientation.w = o.w_quaternion
    artag_pose.orientation.x = o.x_quaternion
    artag_pose.orientation.y = o.y_quaternion
    artag_pose.orientation.z = o.z_quaternion


def get_btn_pose():
    eef_pose = Pose()
    eef_pose.orientation = qa.quat([0,1,0],pi/2)
    eef_pose.position = Point(0,0,0)
    rot = Pose()
    rot.position = Point(0,0,0)
    rot.orientation = qa.quat([0,0,1], -pi/2)
    true_artag_pose = qa.compose_poses(rot, artag_pose)
    return qa.compose_poses(eef_pose, true_artag_pose)


def main():
    rospy.init_node("planner_test_node", anonymous=True)
    rospy.Subscriber("detected_elements", object_list, artag_callback)

    rate = rospy.Rate(25)   # 25hz
    btn_refresh_time = 2
    t = time.time()
    while not rospy.is_shutdown():
        if time.time()-t > btn_refresh_time:
            t = time.time()
            remove_box_pub.publish(Bool())
            time.sleep(.1)
            add_box_pub.publish(get_btn_pose())
        rate.sleep()



if __name__ == "__main__":
    main()
