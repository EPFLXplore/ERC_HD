#!/usr/bin/env python3

import rospy
from task_execution.task_classes import *
import task_execution.quaternion_arithmetic as qa
from task_execution.msg import Task, Object
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Float64MultiArray
import time
from math import pi
from vision_no_ros.msg import panel_object, object_list
import copy

add_box_pub = rospy.Publisher("/arm_control/add_object", Object, queue_size=5)
remove_box_pub = rospy.Publisher("/arm_control/remove_box", Bool, queue_size=5)


end_effector_pose = Pose()
artag_pose = Pose()
artag_pose2 = Pose()


def artag_callback(msg):
    o = msg.detected_objects[0]
    artag_pose.position = o.position
    artag_pose.orientation = o.orientation


def end_effector_callback(msg):
    end_effector_pose.position = msg.position
    end_effector_pose.orientation = msg.orientation


def get_btn_pose():
    obj = Object()
    obj.pose = qa.compose_poses(end_effector_pose, artag_pose)
    obj.type = "box"
    obj.name = "button"
    obj.dims = Float64MultiArray()
    obj.dims.data = [0.2,0.1,0.0001]

    obj3 = Object()
    obj3.dims = Float64MultiArray()
    obj3.dims.data = [0.01, 0.01, 0.1]
    obj3.pose = copy.deepcopy(obj.pose)
    obj3.type = "box"
    obj3.name = "axis"
    axis = qa.point_image([0,0,1], obj.pose.orientation)
    obj3.pose.position = qa.make_point(qa.add(obj3.pose.position, qa.mul(0.05, axis)))
    return obj, obj3
    


def main():
    rospy.init_node("planner_test_node", anonymous=True)
    rospy.Subscriber("/detected_elements", object_list, artag_callback)
    rospy.Subscriber("arm_control/end_effector_pose", Pose, end_effector_callback)

    rate = rospy.Rate(25)   # 25hz
    btn_refresh_time = 4
    t = time.time()
    time.sleep(2)
    while not rospy.is_shutdown():
        if time.time()-t > btn_refresh_time:
            print("eeeeeeeeeeeeeeeee")
            t = time.time()
            o1, o3 = get_btn_pose()
            add_box_pub.publish(o1)
            rospy.sleep(0.2)
            add_box_pub.publish(o3)
            rospy.sleep(0.2)
        rate.sleep()



if __name__ == "__main__":
    main()
