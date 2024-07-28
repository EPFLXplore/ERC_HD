#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from custom_msg.msg import Task
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8

import keyboard
import threading
from typing import Any, Union
from typing_extensions import Self
import math


RectangleLike = Union[list[int], tuple[int, int, int, int]]


class Rectangle:
    def __init__(self, posx: int=0, posy: int=0, dimx: int=0, dimy: int=0):
        self.posx, self.posy, self.dimx, self.dimy = posx, posy, dimx, dimy
    
    @classmethod
    def make(cls, rect: Union[RectangleLike, Self]):
        if isinstance(rect, cls):
            return rect
        if isinstance(rect, (list, tuple)):
            return cls(*rect[:4])
        raise TypeError


class TextBox:
    def __init__(self, hitbox: RectangleLike):
        self.hitbox = Rectangle.make(hitbox)
        


class FakeCSKeyboard(Node):
    def __init__(self):
        super().__init__("fake_cs_keyboard")
        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)


    

    
    def loop(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            self.read_inputs()
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
