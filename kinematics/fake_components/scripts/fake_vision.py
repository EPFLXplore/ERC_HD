#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion, Point
from hd_interfaces.msg import TargetInstruction
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
import kinematics_utils.pose_corrector as pc
import kinematics_utils.pose_tracker as pt
import math


def main():
    rclpy.init()
    node = rclpy.create_node("fake_vision")

    detected_element_pub = node.create_publisher(TargetInstruction, "HD/vision/target_pose", 10)
    node.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)

    try:
        while rclpy.ok():
            # pose = Pose()
            # pose.orientation = qa.quat((1.0, 0.0, 0.0), math.pi)
            # scale = 1000
            # pose.position.x = 0.1 * scale
            # pose.position.y = -0.2 * scale
            # pose.position.z = 0.2 * scale
            #pose.position.x = pose.position.y = 0.0; pose.position.z = 0.00

            #pose = Pose(orientation=qa.quat([1.0, 0.0, 0.0], math.pi))
            # pose = pc.revert_to_vision(pose)   # get it from the perspective of the cameras with their reference
            # msg = TargetInstruction(ar_tag_pose=pose, object_pose=pose)



            pose = Pose()
            pose.orientation = qa.quat((1.0, 0.0, 0.0), math.pi)
            pose.position = Point(x=0.0, y=1.0, z=0.0)
            
            #print(pt.END_EFFECTOR_POSE)
            p = pose
            #pose = qa.compose_poses(pose, qa.reverse_pose(pt.END_EFFECTOR_POSE))
            pose = qa.compose_poses(pose, (pt.END_EFFECTOR_POSE))
            pose = pc.revert_to_vision(pose)   # get it from the perspective of the cameras with their reference

            pose = qan.Pose.make(pose)
            pose.position.x = pose.position.x * 1000
            # pose.position.x *= -1
            # pose.position.y *= -1
            # pose.position.z *= -1
            print((qan.Pose.make(pose)-p).position)
            print(pt.END_EFFECTOR_POSE.position)
            print()
            print()
            #pose = qa.compose_poses(pt.END_EFFECTOR_POSE, pose)
            #pose = pc.revert_to_vision(pose)   # get it from the perspective of the cameras with their reference

            pose = pose.to_ros()
            msg = TargetInstruction(ar_tag_pose=pose, object_pose=pose)
            detected_element_pub.publish(msg)

            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
