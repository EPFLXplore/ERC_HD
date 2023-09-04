#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_corrector as pc
import math


def main():
    rclpy.init()
    node = rclpy.create_node("fake_vision")

    detected_element_pub = node.create_publisher(Pose, "target_pose", 10)

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
            #pose.position.x = pose.position.y = 0.0; pose.position.z = 0.00

            pose = pc.revert_to_vision(pose)   # get it from the perspective of the cameras with their reference
            detected_element_pub.publish(pose)

            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
