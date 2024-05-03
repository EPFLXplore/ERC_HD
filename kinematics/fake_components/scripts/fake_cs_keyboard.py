#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from hd_interfaces.msg import Task
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8

import keyboard
import threading
from typing import Any
import math


MOTOR_COUNT = 8
CONTROL_KEYS = ["a", "z", "e", "r", "t", "y", "u", "i"]


def str_pad(x: Any, length=10) -> str:
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length-len(s))


def normalized(l: list) -> list:
    n = math.sqrt(sum(x**2 for x in l))
    if n == 0:
        return l
    return [x/n for x in l]


class EnumOld:
    # kinda overkill but for fun to mimic a C-style enum + some extra functionalities

    def __init__(self, **kwargs):
        self.items = kwargs
        self.slots = list(kwargs)
        #print(self.slots)
        for k in self.slots:
            setattr(self, k, kwargs[k])

    def __iter__(self):
        for k in self.slots:
            yield self.items[k]
    
    def __len__(self):
        return len(self.slots)
    
    def __getitem__(self, index):
        k = self.slots[index % len(self.slots)]
        return self.items[k]


HDMode = EnumOld(
    MANUAL_INVERSE = 0,
    MANUAL_DIRECT = 1,
    SEMI_AUTONOMOUS = 2,
)


class FakeCSKeyboard(Node):
    def __init__(self):
        super().__init__("fake_cs_keyboard")
        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)

        keyboard.add_hotkey("shift+m+o+d+e", self.change_hd_mode, args=(1,))
        keyboard.add_hotkey("shift+tab+m+o+d+e", self.change_hd_mode, args=(-1,))

        self.hd_mode = HDMode.MANUAL_DIRECT

        self.man_dir_cmd = [0.0]*MOTOR_COUNT
        self.vel = 0.0

        self.man_inv_axis = [0.0, 0.0, 0.0]
        self.man_inv_velocity_scaling = 1.0

        self.semi_auto_cmd = Task.NO_TASK
    
    def change_hd_mode(self, dir):
        self.hd_mode = (self.hd_mode + dir) % len(HDMode)
        msg = Int8(data=self.hd_mode)
        self.mode_change_pub.publish(msg)
    
    def publish_cmd(self):
        if self.hd_mode == HDMode.MANUAL_DIRECT:
            #l = list(map(float, self.man_dir_cmd))
            l = [float(self.vel*c) for c in self.man_dir_cmd]
            print("[", ", ".join(map(str_pad, l)), "]")
            l = [1.0] + l   # add dummy velocity scaling factor
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data = l))
        elif self.hd_mode == HDMode.MANUAL_INVERSE:
            axis = normalized(self.man_inv_axis)
            print("[", ", ".join(map(str_pad, axis)), "]")
            data = [self.man_inv_velocity_scaling] + axis
            self.man_inv_axis_pub.publish(Float32MultiArray(data = data))
        elif self.hd_mode == HDMode.SEMI_AUTONOMOUS:
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NO_TASK:
                return
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NAMED_TARGET:
                msg.str_slot = "optimal_view"
            self.task_pub.publish(msg)
            self.semi_auto_cmd = Task.NO_TASK
    
    def read_inputs(self):
        if self.hd_mode == HDMode.MANUAL_DIRECT:
            self.man_dir_inputs()
        elif self.hd_mode == HDMode.MANUAL_INVERSE:
            self.man_inv_inputs()
        elif self.hd_mode == HDMode.SEMI_AUTONOMOUS:
            self.semi_auto_inputs()

    def man_dir_inputs(self):
        self.man_dir_cmd = [int(keyboard.is_pressed(CONTROL_KEYS[i])) for i in range(MOTOR_COUNT)]
        vel_step = 0.125
        if keyboard.is_pressed("up"):
            self.vel = min(self.vel + vel_step, 1.0)
        if keyboard.is_pressed("down"):
            self.vel = max(self.vel - vel_step, -1.0)
    
    def man_inv_inputs(self):
        control_keys = ["down", "up", "right", "left", "v", "b"]
        self.man_inv_axis = [0.0, 0.0, 0.0]
        for i, key in enumerate(control_keys):
            if keyboard.is_pressed(key):
                self.man_inv_axis[i//2] = (-1)**(i%2)
    
    def semi_auto_inputs(self):
        pass
    
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
