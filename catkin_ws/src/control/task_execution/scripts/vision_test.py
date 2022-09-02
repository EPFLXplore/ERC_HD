#!/usr/bin/env python

import rospy
from task_execution.task_classes import *
import task_execution.quaternion_arithmetic as qa
from task_execution.msg import Task, Object
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Float32MultiArray
import time
from math import pi
from vision_no_ros.msg import panel_object, object_list

add_box_pub = rospy.Publisher("/arm_control/add_object", Object, queue_size=5)
remove_box_pub = rospy.Publisher("/arm_control/remove_box", Bool, queue_size=5)


end_effector_pose = Pose()
artag_pose = Pose()
#artag_pose.orientation = Quaternion(1,0,0,0)


def artag_callback(msg):
    o = msg.detected_objects[0]
    artag_pose.position.x = -o.y_pos/1000
    artag_pose.position.y = -o.x_pos/1000
    artag_pose.position.z = o.z_pos/1000
    artag_pose.orientation.w = o.w_quaternion
    artag_pose.orientation.x = -o.y_quaternion
    artag_pose.orientation.y = o.z_quaternion
    artag_pose.orientation.z = o.x_quaternion


def end_effector_callback(msg):
    global end_effector_pose
    end_effector_pose = msg


def get_btn_pose():
    """eef_pose = Pose()
    eef_pose.orientation = qa.quat([0,1,0],pi/2)
    eef_pose.position = Point(0,0,0)"""
    obj = Object()
    obj.pose = qa.compose_poses(end_effector_pose, artag_pose)
    obj.type = "box"
    obj.name = "button"
    obj.dims = Float32MultiArray()
    obj.dims.data = [0.2,0.1,0.0001]
    return obj
    


def main():
    rospy.init_node("planner_test_node", anonymous=True)
    rospy.Subscriber("/detected_elements", object_list, artag_callback)
    rospy.Subscriber("arm_control/end_effector_pose", Pose, end_effector_callback)

    rate = rospy.Rate(25)   # 25hz
    btn_refresh_time = 2
    t = time.time()
    time.sleep(2)
    while not rospy.is_shutdown():
        if time.time()-t > btn_refresh_time:
            print("eeeeeeeeeeeeeeeee")
            t = time.time()
            add_box_pub.publish(get_btn_pose())
            time.sleep(0.2)
        rate.sleep()



if __name__ == "__main__":
    main()
