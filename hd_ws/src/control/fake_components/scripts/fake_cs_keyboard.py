#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
import keyboard
import threading
import sys


MOTOR_COUNT = 8
CONTROL_KEYS = ["a", "z", "e", "r", "t", "y", "u", "i"]


class FakeCSKeyboard(Node):
    def __init__(self):
        super().__init__("fake_cs_keyboard")
        self.cmd_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_gamepad", 10)

        self.cmd = [0.0]*MOTOR_COUNT
        self.vel = 0.0
    
    def publish_cmd(self):
        msg = Float32MultiArray()
        msg.data = [float(self.vel*c) for c in self.cmd]
        self.cmd_pub.publish(msg)
    
    def get_inputs(self):
        self.cmd = [int((CONTROL_KEYS[i])) for i in range(MOTOR_COUNT)]
        vel_step = 0.125
        if keyboard.is_pressed("up"):
            self.vel = min(self.vel + vel_step, 1.0)
        if keyboard.is_pressed("down"):
            self.vel = max(self.vel - vel_step, -1.0)
    
    def loop(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            self.get_inputs()
            self.publish_cmd()
            rate.sleep()


def main():
    rclpy.init()
    node = FakeCSKeyboard()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()
        

if __name__ == '__main__':
    main()
