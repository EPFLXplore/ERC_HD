#!/usr/bin/env python3

from typing import Tuple
import rclpy
from rclpy.node import Node
import threading
from task_execution.task import *
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_corrector as pc
import kinematics_utils.pose_tracker as pt
from custom_msg.msg import Task, Object, TargetInstruction
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


class Listener:
    def __init__(self):
        self.vect = [0.0] * 3
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
    
    def listen_loop(self):
        while True:
            self.vect = list(map(float, input("new end effector transform: ").split()))
    
    def listen(self):
        self.thread.start()


listener = Listener()
listener.listen()


def artag_callback(msg):
    def extract_pose(source: Pose, dest: Pose):
        pose = Pose()
        pose.position.x = source.position.x/1000
        pose.position.y = source.position.y/1000
        pose.position.z = source.position.z/1000
        pose.orientation = source.orientation
        # pose = pc.correct_vision_pose(pose)
        dest.position = pose.position
        dest.orientation = pose.orientation

    extract_pose(msg.ar_tag_pose, artag_pose)
    extract_pose(msg.object_pose, btn_pose)


def end_effector_callback(msg):
    transform = Pose()
    # transform.orientation = qa.mul(qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2), qa.quat(axis=(0.0, 0.0, 1.0), angle=pi/2))
    transform.orientation = qa.quat(axis=(0.0, 1.0, 0.0), angle=-pi/2)
    vect = [-0.097, -0.7545, -0.3081]
    vect = [0.0, 0.0, 0.2018]
    # vect = listener.vect
    transform.position = qa.point_image(vect, transform.orientation)
    combined = qa.compose_poses(msg, transform)
    # combined = pc.correct_eef_pose(msg)
    end_effector_pose.position = combined.position
    end_effector_pose.orientation = combined.orientation


def get_btn_pose():
    def create_object_representation(pose: Pose, name: str) -> Tuple[Object, Object]:
        body = Object()
        body.pose = qa.compose_poses(end_effector_pose, pose)
        body.type = body.BOX
        body.operation = body.ADD
        body.name = name
        body.shape = Float64MultiArray()
        body.shape.data = [0.1, 0.2, 0.0001]

        axis_obj = Object()
        axis_obj.shape = Float64MultiArray()
        axis_obj.shape.data = [0.01, 0.01, 0.1]
        axis_obj.pose = copy.deepcopy(body.pose)
        body.type = body.BOX
        body.operation = body.ADD
        axis_obj.name = name + "axis"
        axis = qa.point_image([0.0, 0.0, 1.0], body.pose.orientation)
        axis_obj.pose.position = qa.make_point(qa.add(axis_obj.pose.position, qa.mul(0.05, axis)))
        
        return body, axis_obj

    artag_body, artag_axis = create_object_representation(artag_pose, "artag")
    btn_body, btn_axis = create_object_representation(btn_pose, "btn")
    return artag_body, artag_axis, btn_body, btn_axis
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
    node.create_subscription(TargetInstruction, "HD/vision/target_pose", artag_callback, 10)
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
                time.sleep(0.01)
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
