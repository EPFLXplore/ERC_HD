#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool, Float64MultiArray, UInt8
from custom_msg.msg import ArucoObject
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
import kinematics_utils.pose_tracker as pt
import math
from math import pi


def main():
    rclpy.init()
    node = rclpy.create_node("set_camera_offsets_node")

    pub = node.create_publisher(UInt8, "/HD/kinematics/probe_station", 10)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)

    try:
        while rclpy.ok():
            station = int(input("Station to target: "))
            pub.publish(UInt8(data=station))
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
