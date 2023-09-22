#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from task_execution.task import *
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_corrector as pc
import kinematics_utils.pose_tracker as pt
from hd_interfaces.msg import Task, Object, TargetInstruction
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Float64MultiArray
import time
from math import pi
import copy


end_effector_pose = Pose()
artag_pose = Pose()
btn_pose = Pose()

vision_transform = Pose()
vision_transform.orientation = qa.quat([0.0, 0.0, 1.0], pi/2)


def artag_callback(msg):
    pose1 = Pose()
    pose2 = Pose()

    pose1.position.x = msg.ar_tag_pose.position.x/1000
    pose1.position.y = msg.ar_tag_pose.position.y/1000
    pose1.position.z = msg.ar_tag_pose.position.z/1000
    pose1.orientation = msg.ar_tag_pose.orientation
    temp = pc.correct_vision_pose(pose1)
    artag_pose.position = temp.position
    artag_pose.orientation = temp.orientation

    pose2.position.x = msg.object_pose.position.x/1000
    pose2.position.y = msg.object_pose.position.y/1000
    pose2.position.z = msg.object_pose.position.z/1000
    pose2.orientation = msg.object_pose.orientation
    temp = pc.correct_vision_pose(pose2)
    btn_pose.position = temp.position
    btn_pose.orientation = temp.orientation


def end_effector_callback(msg):
    transform = Pose()
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2)
    vect = [-0.097, -0.7545, -0.3081]
    transform.position = qa.point_image(vect, transform.orientation)
    combined = qa.compose_poses(msg, transform)
    #combined = pc.correct_eef_pose(msg)
    end_effector_pose.position = combined.position
    end_effector_pose.orientation = combined.orientation


def get_btn_pose():
    obj = Object()
    obj.pose = qa.compose_poses(end_effector_pose, artag_pose)
    obj.type = obj.BOX
    obj.operation = obj.ADD
    obj.name = "artag"
    obj.shape = Float64MultiArray()
    obj.shape.data = [0.2, 0.1, 0.0001]

    obj3 = Object()
    obj3.shape = Float64MultiArray()
    obj3.shape.data = [0.01, 0.01, 0.1]
    obj3.pose = copy.deepcopy(obj.pose)
    obj.type = obj.BOX
    obj.operation = obj.ADD
    obj3.name = "artag_axis"
    axis = qa.point_image([0.0, 0.0, 1.0], obj.pose.orientation)
    obj3.pose.position = qa.make_point(qa.add(obj3.pose.position, qa.mul(0.05, axis)))

    # obj2 = copy.deepcopy(obj)
    # obj2.name = "btn"
    # obj2.pose = qa.compose_poses(end_effector_pose, btn_pose)

    # obj4 = copy.deepcopy(obj3)
    # obj4.name = "btn_axis"
    # obj4.pose = copy.deepcopy(obj2.pose)
    # axis = qa.point_image([0.0, 0.0, 1.0], obj2.pose.orientation)
    # obj4.pose.position = qa.make_point(qa.add(obj4.pose.position, qa.mul(0.05, axis)))

    obj2 = Object()
    obj2.pose = qa.compose_poses(end_effector_pose, btn_pose)
    obj.type = obj.BOX
    obj.operation = obj.ADD
    obj2.name = "btn"
    obj2.shape = Float64MultiArray()
    obj2.shape.data = [0.2, 0.1, 0.0001]

    obj4 = Object()
    obj4.shape = Float64MultiArray()
    obj4.shape.data = [0.01, 0.01, 0.1]
    obj4.pose = copy.deepcopy(obj2.pose)
    obj.type = obj.BOX
    obj.operation = obj.ADD
    obj4.name = "btn_axis"
    axis = qa.point_image([0.0, 0.0, 1.0], obj2.pose.orientation)
    obj4.pose.position = qa.make_point(qa.add(obj4.pose.position, qa.mul(0.05, axis)))

    obj.pose = pc.revert_from_vision(obj.pose)
    obj2.pose = pc.revert_from_vision(obj2.pose)
    obj3.pose = pc.revert_from_vision(obj3.pose)
    obj4.pose = pc.revert_from_vision(obj4.pose)

    return obj, obj2, obj3, obj4


def main():
    rclpy.init()
    node = rclpy.create_node("kinematics_vision_test")

    node.create_subscription(Pose, "/HD/kinematics/eef_pose", end_effector_callback, 10)
    node.create_subscription(TargetInstruction, "target_pose", artag_callback, 10)
    #node.create_subscription(PanelObject, "/HD/vision/distance_topic", pt.detected_object_pose_callback, 10)

    add_object_pub = node.create_publisher(Object, "/HD/kinematics/add_object", 10)
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(30)
    btn_refresh_time = 0.1
    t = time.time()
    try:
        while rclpy.ok():
            if time.time()-t > btn_refresh_time:
                t = time.time()
                o1, o2, o3, o4 = get_btn_pose()
                add_object_pub.publish(o1)
                time.sleep(0.01)
                add_object_pub.publish(o2)
                # time.sleep(0.01)
                # add_object_pub.publish(o3)
                # time.sleep(0.01)
                # add_object_pub.publish(o4)
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()