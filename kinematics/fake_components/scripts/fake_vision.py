#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion
from custom_msg.msg import TargetInstruction
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
import kinematics_utils.pose_tracker as pt
import math
from math import pi


def generate_points():
    def random_in_range(lower: float, upper: float) -> float:
        import random
        x = random.random() * (upper-lower) + lower
        return x
    
    
    x = [random_in_range(0.57, 1.0) for _ in range(3)]
    y = [random_in_range(-0.5, 0.2) for _ in range(3)]
    z = [random_in_range(0, 0.6) for _ in range(3)]
    
    
    def combinations(lists, current=0):
        if current == len(lists):
            yield tuple()
            return
        for x in lists[current]:
            for comb in combinations(lists, current+1):
                yield (x,) + comb
    
    
    def format_point(p):
        return " ".join(map(str, p))
    
    
    combs = list(map(format_point, combinations([x, y, z])))
    for p in combs: print(p)



def generate_points():
    def random_in_range(lower: float, upper: float) -> float:
        import random
        x = random.random() * (upper-lower) + lower
        return x
    
    x = random_in_range(0.8, 1.2)
    y = random_in_range(-0.5, 0.2)
    z = random_in_range(0, 0.6)
    print(x, y, z)


class Listener:
    def __init__(self):
        self.vect = [0.0] * 3
        self.vect = [0.6, -0.26, 0.59]
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)
    
    def listen_loop(self):
        while True:
            self.vect = list(map(float, input("New fake vision object pos: ").split()))
    
    def listen(self):
        self.thread.start()


listener = Listener()
listener.listen()


def main():
    absolute = True
    
    rclpy.init()
    node = rclpy.create_node("fake_vision")

    node.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)
    detected_element_pub = node.create_publisher(TargetInstruction, "/HD/perception/button_pose", 10)

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(5)

    try:
        while rclpy.ok():
            pose = qan.Pose()
            # pose.orientation = qa.mul(qa.quat((0.0, 0.0, 1.0), math.pi/2), qa.quat((0.0, 1.0, 0.0), -math.pi/2))
            # pose.orientation = qan.Quaternion.from_axis_angle((0, 0, 1), pi/2) * qan.Quaternion.from_axis_angle((0, 1, 0), -pi/2)
            pose.orientation = qan.Quaternion.from_axis_angle((0, 1, 0), -pi/2) * qan.Quaternion.from_axis_angle((0, 0, 1), -pi/2)
            scale = 1000
            pose.position.x = listener.vect[0]
            pose.position.y = listener.vect[1]
            pose.position.z = listener.vect[2]
            # pose = qan.Pose()
            if absolute: pose = pc.abs_to_vision(pose)
            pose.position *= scale

            pose = pose.publishable()
            msg = TargetInstruction(ar_tag_pose=pose, object_pose=pose)
            detected_element_pub.publish(msg)

            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
