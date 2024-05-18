#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion
from hd_interfaces.msg import TargetInstruction
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_corrector as pc
import math


def main():
    rclpy.init()
    node = rclpy.create_node("fake_vision")

    detected_element_pub = node.create_publisher(TargetInstruction, "HD/vision/target_pose", 10)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)

    try:
        while rclpy.ok():
            pose = Pose()
            pose.orientation = qa.quat((1.0, 0.0, 0.0), math.pi)
            scale = 1000
            pose.position.x = 0.1 * scale
            pose.position.y = -0.2 * scale
            pose.position.z = 0.2 * scale
            #pose.position.x = pose.position.y = pose.position.z = 0.0

            #pose = Pose(orientation=qa.quat([1.0, 0.0, 0.0], math.pi))
            pose = Pose()
            rev = qa.reverse_pose(pc.CAMERA_TRANSFORM)
            #pose = qa.compose_multiple_poses(pose, rev)
            pose = pc.revert_to_vision(pose)   # get it from the perspective of the cameras with their reference
            pose.position.x *= scale
            pose.position.y *= scale
            pose.position.z *= scale
            node.get_logger().info(str(pose))
            msg = TargetInstruction(ar_tag_pose=pose, object_pose=pose)
            detected_element_pub.publish(msg)

            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
