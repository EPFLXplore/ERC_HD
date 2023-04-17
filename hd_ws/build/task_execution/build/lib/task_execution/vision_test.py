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
import copy

add_box_pub = rospy.Publisher("/arm_control/add_object", Object, queue_size=5)
remove_box_pub = rospy.Publisher("/arm_control/remove_box", Bool, queue_size=5)


end_effector_pose = Pose()
artag_pose = Pose()
artag_pose2 = Pose()
#artag_pose.orientation = Quaternion(1,0,0,0)


def artag_callback(msg):
    o = msg.detected_objects[0]
    artag_pose.position.x = -o.y_pos/1000
    artag_pose.position.y = -o.x_pos/1000
    artag_pose.position.z = o.z_pos/1000
    #artag_pose.orientation.w = o.w_quaternion
    #artag_pose.orientation.x = -o.y_quaternion
    #artag_pose.orientation.y = o.x_quaternion
    #artag_pose.orientation.z = o.z_quaternion

    artag_pose.orientation.x, artag_pose.orientation.y, artag_pose.orientation.z, artag_pose.orientation.w = (o.z_quaternion, -o.w_quaternion, o.y_quaternion, o.x_quaternion)

    """axis = qa.point_image([0,1,0], artag_pose.orientation)
    q = qa.quat(axis, pi)
    artag_pose.orientation = qa.mul(q, artag_pose.orientation)"""

    artag_pose2.position.x = -o.y_pos_tag/1000
    artag_pose2.position.y = -o.x_pos_tag/1000
    artag_pose2.position.z = o.z_pos_tag/1000
    artag_pose2.orientation = copy.deepcopy(artag_pose.orientation)

    dx = (artag_pose2.position.x - artag_pose.position.x)
    dy = -(artag_pose2.position.y - artag_pose.position.y)
    dz = -(artag_pose2.position.z - artag_pose.position.z)

    position = artag_pose2.position
    q = qa.mul(dx, qa.point_image([1,0,0], artag_pose2.orientation))
    position = qa.add(position, q)
    q = qa.mul(dy, qa.point_image([0,1,0], artag_pose2.orientation))
    position = qa.add(position, q)
    q = qa.mul(dz, qa.point_image([0,0,1], artag_pose2.orientation))
    position = qa.add(position, q)
    artag_pose.position = qa.make_point(position)

    artag_pose2.orientation = qa.turn_around(artag_pose2.orientation, [0,0,1])
    artag_pose.orientation = copy.deepcopy(artag_pose2.orientation)


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

    obj2 = Object()
    obj2.pose = qa.compose_poses(end_effector_pose, artag_pose2)
    obj2.type = "box"
    obj2.name = "artag"
    obj2.dims = Float32MultiArray()
    obj2.dims.data = [0.2,0.1,0.0001]

    obj3 = Object()
    obj3.dims = Float32MultiArray()
    obj3.dims.data = [0.01, 0.01, 0.1]
    obj3.pose = copy.deepcopy(obj2.pose)
    obj3.type = "box"
    obj3.name = "axis"
    axis = qa.point_image([0,0,1], obj2.pose.orientation)
    obj3.pose.position = qa.make_point(qa.add(obj3.pose.position, qa.mul(0.05, axis)))
    return obj, obj2, obj3
    


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
            o1, o2, o3 = get_btn_pose()
            add_box_pub.publish(o1)
            rospy.sleep(0.2)
            add_box_pub.publish(o2)
            rospy.sleep(0.2)
            add_box_pub.publish(o3)
            rospy.sleep(0.2)
        rate.sleep()



if __name__ == "__main__":
    main()
