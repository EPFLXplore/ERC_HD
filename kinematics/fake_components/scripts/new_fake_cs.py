#!/usr/bin/env python3

from typing import Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import evdev.events
import threading
from time import sleep
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8, Bool
from custom_msg.msg import Task
from custom_msg.srv import HDMode
import math
import itertools
from collections.abc import Callable
from typing import List, Dict, Any, Union
# from enum_test import Enum


def clean(x: float, tol=0.05) -> float:
    return 0.0 if abs(x) < tol else x


def str_pad(x: Any, length=10) -> str:
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length-len(s))


def add(l1: list, l2: list) -> list:
    return [x+y for x, y in zip(l1, l2)]


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
        self.slot_values = [kwargs[slot] for slot in self.slots]
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
    
    def next(self, val):
        i = (self.slot_values.index(val)+1) % len(self.slots)
        return self.slot_values[i]


Mode = EnumOld(
    IDLE = -1,
    MANUAL_DIRECT = 1,
    SEMI_AUTONOMOUS = 2,
    MANUAL_INVERSE = 0,
    #AUTONOMOUS = 3
)


SemiAutoTask = EnumOld(
    NO_TASK = Task.NO_TASK,
    BTN_TASK = Task.BUTTON,
    PLUG_VOLTMETER = Task.PLUG_VOLTMETER_APPROACH,
    NAMED_TARGET_TASK = Task.NAMED_TARGET
)


def map_param(node: Node, name: str,choices: Dict[str, Any], default: str = "", ):
    node.declare_parameter(name, default)    # TODO: figure out how to pass a dict parameter and not use eval!
    value = node.get_parameter(name).get_parameter_value().string_value
    if not value in choices:
        raise ValueError("invalid value for parameter")
    return choices[value]


class ControlStation(Node):
    """
    Class reading gamepad and sending commands (to handling device) accordingly
    """

    def __init__(self):
        super().__init__("fake_cs_gamepad")
        
        keyboard_control = map_param(self, "input_device", {"gamepad": False, "keyboard": True})

        self.vel_cmd = [0.0]*8
        self.axis_cmd = [0.0]*3
        self.man_inv_twist = Twist()
        self.man_inv_velocity_scaling = 1.0
        self.semi_auto_cmd = Task.NO_TASK

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = Mode.IDLE

        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, self.get_param("rover_hd_man_dir_topic"), 10)
        self.man_inv_twist_pub = self.create_publisher(Float32MultiArray, self.get_param("rover_hd_man_inv_topic"), 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        # self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        self.mode_cli = self.create_client(HDMode, self.get_param("hd_fsm_mode_srv"))

        if keyboard_control:
            from input_handling.keyboard import KeyboardConfig
            self.input_config = KeyboardConfig()
        else:
            from input_handling.gamepad import GamePadConfig
            self.input_config = GamePadConfig()
        self.input_config.background_loop()
    
        self.create_bindings(keyboard_control=keyboard_control)
        
        self.timer_period = 1/30
        self.timer = self.create_timer(self.timer_period, self.publish_cmd)

    def get_param(self, name: str, default: str = ""):
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().string_value
    
    def create_bindings(self, keyboard_control: bool = False):
        if keyboard_control:
            self.create_keyboard_bindings()
        else:
            self.create_gamepad_bindings()
            
    def create_keyboard_bindings(self):
        from input_handling.keyboard import KeyboardConfig
        self.input_config.bind(KeyboardConfig.m, self.switch_mode, "do")
        
        for i, input in enumerate([KeyboardConfig.a, KeyboardConfig.z, KeyboardConfig.e, KeyboardConfig.r, KeyboardConfig.t, KeyboardConfig.y]):
            self.input_config.bind(input, self.set_manual_velocity, "value", joint_index=i)
        
        for i, input in enumerate([KeyboardConfig.A, KeyboardConfig.Z, KeyboardConfig.E, KeyboardConfig.R, KeyboardConfig.T, KeyboardConfig.Y]):
            self.input_config.bind(input, self.set_manual_velocity, "value", joint_index=i, multiplier=-1)
    
        self.input_config.bind(KeyboardConfig.U, self.set_man_inv_axis, "value", coordinate=0, multiplier=1)
        self.input_config.bind(KeyboardConfig.u, self.set_man_inv_axis, "value", coordinate=0, multiplier=-1)
        self.input_config.bind(KeyboardConfig.down, self.set_man_inv_axis, "value", coordinate=1, multiplier=1)
        self.input_config.bind(KeyboardConfig.up, self.set_man_inv_axis, "value", coordinate=1, multiplier=-1)
        self.input_config.bind(KeyboardConfig.left, self.set_man_inv_axis, "value", coordinate=2, multiplier=1)
        self.input_config.bind(KeyboardConfig.right, self.set_man_inv_axis, "value", coordinate=2, multiplier=-1)
        
        self.input_config.bind(KeyboardConfig._7, self.set_man_inv_angular, "value", coordinate=0, multiplier=1)
        self.input_config.bind(KeyboardConfig._9, self.set_man_inv_angular, "value", coordinate=0, multiplier=-1)
        self.input_config.bind(KeyboardConfig._4, self.set_man_inv_angular, "value", coordinate=1, multiplier=1)
        self.input_config.bind(KeyboardConfig._6, self.set_man_inv_angular, "value", coordinate=1, multiplier=-1)
        self.input_config.bind(KeyboardConfig._1, self.set_man_inv_angular, "value", coordinate=2, multiplier=1)
        self.input_config.bind(KeyboardConfig._3, self.set_man_inv_angular, "value", coordinate=2, multiplier=-1)
    
    def create_gamepad_bindings(self):
        from input_handling.gamepad import GamePadConfig
        
        # ==== mode switch ====
        self.input_config.bind(GamePadConfig.PS, self.switch_mode, "do")
        
        # ==== manual direct ====
        inputs = [GamePadConfig.RH, GamePadConfig.RV, GamePadConfig.R2, GamePadConfig.L2, GamePadConfig.LV, GamePadConfig.LH]
        multipliers = [-1, -1, -1, 1, -1, 1]
        for i, (input, mult) in enumerate(zip(inputs, multipliers)):
            self.input_config.bind(input, self.set_manual_velocity, "value", joint_index=i, multiplier=mult)
        self.input_config.bind(GamePadConfig.R1, self.flip_manual_velocity_dir, joint_index=2)
        self.input_config.bind(GamePadConfig.L1, self.flip_manual_velocity_dir, joint_index=3)
        # gripper
        for val, input in zip([1.0, -1.0, 0.1, -0.1], [GamePadConfig.CIRCLE, GamePadConfig.SQUARE, GamePadConfig.TRIANGLE, GamePadConfig.CROSS]):
            self.input_config.bind(input, self.set_gripper_speed, "event_value", value=val)

        # ==== semi auto ====
        for i, input in enumerate([GamePadConfig.L2, GamePadConfig.R2, GamePadConfig.CIRCLE, GamePadConfig.SQUARE, GamePadConfig.TRIANGLE, GamePadConfig.CROSS]):
            #self.input_config.bind(input, self.set_man_inv_axis, "value", coordinate=i//2, multiplier=(-1)**i)
            self.input_config.bind(input, self.set_semi_auto_cmd, "event_value", index=i)
    
        # ==== manual inverse ====
        self.input_config.bind(GamePadConfig.RV, self.set_man_inv_axis, "value", coordinate=0, multiplier=1)
        self.input_config.bind(GamePadConfig.RH, self.set_man_inv_axis, "value", coordinate=2, multiplier=-1)
        self.input_config.bind(GamePadConfig.R2, self.set_man_inv_axis, "value", coordinate=1, multiplier=-1)
        self.input_config.bind(GamePadConfig.L2, self.set_man_inv_axis, "value", coordinate=1, multiplier=1)
        
        self.input_config.bind(GamePadConfig.LH, self.set_man_inv_angular, "value", coordinate=0, multiplier=1)
        self.input_config.bind(GamePadConfig.LV, self.set_man_inv_angular, "value", coordinate=2, multiplier=-1)
        self.input_config.bind(GamePadConfig.L1, self.set_man_inv_angular, "value", coordinate=1, multiplier=1)
        self.input_config.bind(GamePadConfig.R1, self.set_man_inv_angular, "value", coordinate=1, multiplier=-1)
        
    def publish_cmd(self):
        verbose = True
        if self.hd_mode == Mode.IDLE:
            return
        elif self.hd_mode == Mode.MANUAL_DIRECT:
            l = list(map(clean, map(float, self.vel_cmd)))
            l[2] *= self.joint3_dir
            l[3] *= self.joint4_dir
            if verbose:
                print("[", ", ".join(map(str_pad, l)), "]")
            # l = [1.0] + l   # add dummy velocity scaling factor
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data=l))
        elif self.hd_mode == Mode.MANUAL_INVERSE:
            if verbose:
                print(self.man_inv_twist)
            linear = self.man_inv_twist.linear
            angular = self.man_inv_twist.angular
            gripper_torque = 0.0   # TODO
            data = [linear.x, linear.y, linear.z, angular.x, angular.y, angular.z, gripper_torque]
            self.man_inv_twist_pub.publish(Float32MultiArray(data=data))
        elif self.hd_mode == Mode.SEMI_AUTONOMOUS:
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NO_TASK:
                return
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NAMED_TARGET:
                msg.str_slot = "optimal_view"
            self.task_pub.publish(msg)
            self.semi_auto_cmd = Task.NO_TASK
    
    @staticmethod
    def restrict_mode(target_mode: int):
        def ensure_mode_decorator(func: Callable) -> Callable:
            def wrapper(*args, **kwargs) -> Any:
                cs: ControlStation = args[0]
                if cs.hd_mode != target_mode:
                    return
                return func(*args, **kwargs)
            return wrapper
        return ensure_mode_decorator

    def switch_mode(self, do: int = 1):
        if not do:
            return
        self.hd_mode = Mode.next(self.hd_mode)
        req = HDMode.Request(mode=self.hd_mode)
        future = self.mode_cli.call_async(req)
        # TODO: check result

    @restrict_mode(Mode.MANUAL_DIRECT)
    def set_manual_velocity(self, joint_index: int, value: float, multiplier: int = 1):
        self.vel_cmd[joint_index] = value * multiplier
    
    @restrict_mode(Mode.MANUAL_DIRECT)
    def flip_manual_velocity_dir(self, joint_index: int):
        if joint_index == 2:
            self.joint3_dir *= -1
        elif joint_index == 3:
            self.joint4_dir *= -1
    
    @restrict_mode(Mode.MANUAL_DIRECT)
    def set_gripper_speed(self, value: float, event_value: float):
        self.vel_cmd[6] = value * event_value
    
    @restrict_mode(Mode.MANUAL_DIRECT)
    def set_rassor_speed(self, value, event_value):
        self.vel_cmd[7] = value * event_value
    
    @restrict_mode(Mode.MANUAL_INVERSE)
    def set_man_inv_axis(self, coordinate: int, value: float, multiplier: int = 1):
        v = clean(value * multiplier)
        if coordinate == 0:
            self.man_inv_twist.linear.x = v
        elif coordinate == 1:
            self.man_inv_twist.linear.y = v
        else:
            self.man_inv_twist.linear.z = v
    
    @restrict_mode(Mode.MANUAL_INVERSE)
    def set_man_inv_angular(self, coordinate: int, value: float, multiplier: int = 1):
        v = clean(value * multiplier)
        if coordinate == 0:
            self.man_inv_twist.angular.x = v
        elif coordinate == 1:
            self.man_inv_twist.angular.y = v
        else:
            self.man_inv_twist.angular.z = v
    
    @restrict_mode(Mode.SEMI_AUTONOMOUS)
    def set_semi_auto_cmd(self, index: int, event_value: float):
        if event_value != 1: return
        self.semi_auto_cmd = SemiAutoTask[index]
    
    @restrict_mode(Mode.MANUAL_INVERSE)
    def set_man_inv_velocity_scaling(self, value: float):
        self.man_inv_velocity_scaling = 1.0 - abs(value)


def main():
    rclpy.init()
    node = ControlStation()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
