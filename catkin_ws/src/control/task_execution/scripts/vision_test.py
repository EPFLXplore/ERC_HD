#!/usr/bin/env python

import rospy
from task_execution.task_classes import *
import task_execution.quaternion_arithmetic as qa
from task_execution.msg import Task
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
import time
from math import pi

add_box_pub = rospy.Publisher("/arm_control/world_update", Pose)
remove_box_pub = rospy.Publisher("/arm_control/remove_box", Bool)


#end_effector_pose = Pose()
ARtag_pose = Pose()


def artag_callback(msg):



def get_btn_pose():
    eef_pose = Pose()
    eef_pose.orientation = qa.quat([0,1,0],pi/2)
    rot = Pose()
    rot.position = Point(0,0,0)
    rot.orientation = qa.quat([0,0,1], -pi/2)
    true_artag_pose = qa.compose_poses(rot, ARtag_pose)
    return qa.compose(eef_pose, true_artag_pose)


def main():
    rospy.init_node("planner_test_node", anonymous=True)
    rospy.Subscriber("", , self.artag_callback)

    rate = rospy.Rate(25)   # 25hz
    btn_refresh_time = 5
    t = time.time()
    while not rospy.is_shutdown():
        if time.time()-t > btn_refresh_time:
            remove_box_pub.publish(Bool())
            time.sleep(.5)
            add_box_pub.publish(get_btn_pose())
        rate.sleep()




