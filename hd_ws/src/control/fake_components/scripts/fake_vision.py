#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose
import trajectory_planner.quaternion_arithmetic as qa


def main():
    rclpy.init()
    node = rclpy.create_node("fake_vision")

    detected_element_pub = node.create_publisher(Pose, "/HD/detected_element", 10)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)

    try:
        while rclpy.ok():
            pose = Pose()
            pose.orientation = qa.quat((1.0, 0.0, 0.0), 4)#math.pi)
            pose.position.x = 0.1
            pose.position.y = -0.3
            pose.position.z = 0.17
            #pose.position.x = pose.position.y = 0.0; pose.position.z = 0.00
            detected_element_pub.publish(pose)

            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
