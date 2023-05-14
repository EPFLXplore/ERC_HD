#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from trajectory_planner.task_classes import *
import trajectory_planner.quaternion_arithmetic as qa
from kerby_interfaces.msg import Task, Object
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Float64MultiArray
import time
from math import pi
import copy


end_effector_pose = Pose()
artag_pose = Pose()
artag_pose2 = Pose()


def artag_callback(msg):
    artag_pose.position = msg.position
    artag_pose.orientation = msg.orientation


def end_effector_callback(msg):
    end_effector_pose.position = msg.position
    end_effector_pose.orientation = msg.orientation


def get_btn_pose():
    obj = Object()
    obj.pose = qa.compose_poses(end_effector_pose, artag_pose)
    obj.type = "box"
    obj.name = "button"
    obj.shape = Float64MultiArray()
    obj.shape.data = [0.2, 0.1, 0.0001]

    obj3 = Object()
    obj3.shape = Float64MultiArray()
    obj3.shape.data = [0.01, 0.01, 0.1]
    obj3.pose = copy.deepcopy(obj.pose)
    obj3.type = "box"
    obj3.name = "axis"
    axis = qa.point_image([0.0, 0.0, 1.0], obj.pose.orientation)
    obj3.pose.position = qa.make_point(qa.add(obj3.pose.position, qa.mul(0.05, axis)))
    return obj, obj3


def main():
    rclpy.init()
    node = rclpy.create_node("kinematics_vision_test")

    node.create_subscription(Pose, "/HD/kinematics/eef_pose", end_effector_callback, 10)
    node.create_subscription(Pose, "/HD/detected_element", artag_callback, 10)

    add_object_pub = node.create_publisher(Object, "/HD/kinematics/add_object", 10)
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)
    btn_refresh_time = 0.1
    t = time.time()
    try:
        while rclpy.ok():
            if time.time()-t > btn_refresh_time:
                t = time.time()
                o1, o3 = get_btn_pose()
                add_object_pub.publish(o1)
                time.sleep(0.01)
                add_object_pub.publish(o3)
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()