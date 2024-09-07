#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool, Float64MultiArray
from custom_msg.msg import ArucoObject
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
import kinematics_utils.pose_tracker as pt
import math
from math import pi


class Listener:
    def __init__(self, pub):
        p = pc.CAMERA_TRANSFORM.position
        self.vect = [p.x, p.y, p.z]
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.pub = pub
    
    def listen_loop(self):
        while True:
            self.vect = list(map(float, input(f"Current offset: {self.vect}. New offset: ").split()))
            msg = Float64MultiArray(data=self.vect)
            self.pub.publish(msg)
    
    def listen(self):
        self.thread.start()


def main():
    rclpy.init()
    node = rclpy.create_node("set_camera_offsets_node")

    pub = node.create_publisher(ArucoObject, "/HD/kinematics/set_camera_transform", 10)
    listener = Listener(pub)
    listener.listen()


    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)

    try:
        while rclpy.ok():
            
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
