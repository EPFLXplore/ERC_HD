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
from hd_interfaces.msg import Task, MotorCommand
import math
import itertools
from collections.abc import Callable
from typing import List, Dict, Any, Union


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
        i = (self.slots.index(val)+1) % len(self.slots)
        return self.slots[i]



def Enum(**kwargs):
    """
    *** very overkill and very useless but I was bored ***
    Tries to mimic a C-style enum with some additional useful properties.
    :param kwargs: the members of the enum
    :return: A class type having as class attributes instances of that class corresponding to the members of the enum.
        Thanks to the class being constructed on a custom metaclass, iteration, len computation and item query can be performed directly on the enum class object.
    """
    if len(kwargs) == 0:
        ValType = int   # by default
    else:
        ValType = type(list(kwargs.values())[0])

    class EnumMetaClass(type):        
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.__SLOTS = []

        def __iter__(self):
            for obj in self.__SLOTS:
                yield obj
        
        def __len__(self):
            return len(self.__SLOTS)
        
        def __getitem__(self, index: int):
            obj = self.__SLOTS[index % len(self.__SLOTS)]
            return obj
        
        def next(self, obj):
            for i, slot in enumerate(self.__SLOTS):
                if slot == obj:
                    break
            return self.__SLOTS[(i+1)%len(self.__SLOTS)]
        
        def register_instance(self, name: str, value: ValType):
            setattr(self, name, value)
            self.__SLOTS.append(getattr(self, name))

    class EnumClassTemplate(metaclass=EnumMetaClass):
        def __init__(self, name: str, value: ValType):
            self.name = name
            self.value = value
            type(self).register_instance(name, self)

        def __eq__(self, other: Any):
            if isinstance(other, type(self)):
                return other.value == self.value
            elif isinstance(other, type(self.value)):
                return other == self.value
            return False

        def next(self):
            return type(type(self)).next(type(self), self)

        def __repr__(self):
            return f"<name: {self.name}, value: {self.value}>"

    EnumClass = EnumMetaClass(
        "EnumClass", 
        (object,), 
        {"__init__": EnumClassTemplate.__init__, "__eq__": EnumClassTemplate.__eq__, "__repr__": EnumClassTemplate.__repr__, "next": EnumClassTemplate.next}
    )
    for name, value in kwargs.items():
        EnumClass(name, value)
    EnumClass.__new__ = lambda *args, **kwargs: None    # class won't be instantiable anymore
    return EnumClass


HDMode = EnumOld(
    IDLE = -1,
    MANUAL_INVERSE = 0,
    MANUAL_DIRECT = 1,
    SEMI_AUTONOMOUS = 2,
    #AUTONOMOUS = 3
)

HDSubMode = EnumOld(
    JOINTSPACE = 0,
    CARTESIAN = 1,
)


SemiAutoTask = EnumOld(
    NO_TASK = Task.NO_TASK,
    BTN_TASK = Task.BUTTON,
    PLUG_VOLTMETER = Task.PLUG_VOLTMETER_APPROACH,
    NAMED_TARGET_TASK = Task.NAMED_TARGET
)


class ControlStation(Node):
    """
    Class reading gamepad and sending commands (to handling device) accordingly
    """

    def __init__(self):
        super().__init__("fake_cs_gamepad")
        
        keyboard_control = True

        self.vel_cmd = [0.0]*8
        self.torque_scaling_factor = 0.0
        self.axis_cmd = [0.0]*3
        #self.man_inv_axis = [0.0]*3
        self.man_inv_twist = Twist()
        self.man_inv_velocity_scaling = 1.0
        self.semi_auto_cmd = Task.NO_TASK

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = HDMode.MANUAL_INVERSE #HDMode.MANUAL_DIRECT
        self.hd_sub_mode = HDSubMode.JOINTSPACE

        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        #self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.man_inv_twist_pub = self.create_publisher(Twist, "/ROVER/HD_man_inv_twist", 10)
        self.man_inv_joint_pub = self.create_publisher(Float64MultiArray, "/ROVER/HD_man_inv_joint", 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        self.sub_mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_sub_mode", 10)
        self.gripper_torque_pub = self.create_publisher(MotorCommand, "HD/kinematics/single_joint_cmd", 10)

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

    def create_bindings(self, keyboard_control: bool = False):
        if keyboard_control:
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
            # left -> down
            # right -> up
            # up -> U
            # down -> u
            # u -> left
            # U -> right
        else:
            from input_handling.gamepad import GamePadConfig
            for i, input in enumerate([GamePadConfig.RH, GamePadConfig.RV, GamePadConfig.R2, GamePadConfig.L2, GamePadConfig.LV, GamePadConfig.LH]):
                self.input_config.bind(input, self.set_manual_velocity, "value", joint_index=i)

            self.input_config.bind(GamePadConfig.R1, self.flip_manual_velocity_dir, "value", joint_index=2)
            self.input_config.bind(GamePadConfig.L1, self.flip_manual_velocity_dir, "value", joint_index=3)
            self.input_config.bind(GamePadConfig.PS, self.switch_mode, "do")

            for val, input in zip([1.0, -1.0, 0.1, -0.1], [GamePadConfig.CIRCLE, GamePadConfig.SQUARE, GamePadConfig.TRIANGLE, GamePadConfig.CROSS]):
                self.input_config.bind(input, self.set_gripper_speed, "event_value", value=val)

            self.input_config.bind(GamePadConfig.DIRH, self.set_rassor_speed, "event_value", value=1.0)
            self.input_config.bind(GamePadConfig.DIRV, self.set_rassor_speed, "event_value", value=-0.1)

            for i, input in enumerate([GamePadConfig.L2, GamePadConfig.R2, GamePadConfig.CIRCLE, GamePadConfig.SQUARE, GamePadConfig.TRIANGLE, GamePadConfig.CROSS]):
                #self.input_config.bind(input, self.set_man_inv_axis, "value", coordinate=i//2, multiplier=(-1)**i)
                self.input_config.bind(input, self.set_semi_auto_cmd, "event_value", index=i)
        
            
            self.input_config.bind(GamePadConfig.L2, self.set_man_inv_axis, "value", coordinate=0, multiplier=1)
            self.input_config.bind(GamePadConfig.R2, self.set_man_inv_axis, "value", coordinate=0, multiplier=-1)
            self.input_config.bind(GamePadConfig.RH, self.set_man_inv_axis, "value", coordinate=1, multiplier=1)
            self.input_config.bind(GamePadConfig.RV, self.set_man_inv_axis, "value", coordinate=2, multiplier=-1)
            
            self.input_config.bind(GamePadConfig.L1, self.set_man_inv_angular, "value", coordinate=0, multiplier=1)
            self.input_config.bind(GamePadConfig.R1, self.set_man_inv_angular, "value", coordinate=0, multiplier=-1)
            self.input_config.bind(GamePadConfig.LV, self.set_man_inv_angular, "value", coordinate=1, multiplier=-1)
            self.input_config.bind(GamePadConfig.LH, self.set_man_inv_angular, "value", coordinate=2, multiplier=-1)
        
        #self.input_config.bind(GamePadConfig.LV, self.set_man_inv_velocity_scaling, "value")

    def publish_cmd(self):
        verbose = True
        if self.hd_mode == HDMode.IDLE:
            return
        if self.torque_scaling_factor != 0:    # gripper
            msg = MotorCommand(
                name = "Gripper",
                mode = MotorCommand.TORQUE,
                command = self.torque_scaling_factor
            )
            self.gripper_torque_pub.publish(msg)
        if self.hd_mode == HDMode.MANUAL_DIRECT:
            l = list(map(clean, map(float, self.vel_cmd)))
            l[2] *= self.joint3_dir
            l[3] *= self.joint4_dir
            if verbose:
                print("[", ", ".join(map(str_pad, l)), "]")
            l = [1.0] + l   # add dummy velocity scaling factor
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data = l))
        elif self.hd_mode == HDMode.MANUAL_INVERSE:
            if self.hd_sub_mode == HDSubMode.JOINTSPACE:
                l = list(map(clean, map(float, self.vel_cmd)))
                l[2] *= self.joint3_dir
                l[3] *= self.joint4_dir
                if verbose:
                    print("[", ", ".join(map(str_pad, l)), "]")
                self.man_inv_joint_pub.publish(Float64MultiArray(data=l))
            else:
                if verbose:
                    print(self.man_inv_twist)
                self.man_inv_twist_pub.publish(self.man_inv_twist)
        elif self.hd_mode == HDMode.SEMI_AUTONOMOUS:
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NO_TASK:
                return
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NAMED_TARGET:
                msg.str_slot = "optimal_view"
            self.task_pub.publish(msg)
            self.semi_auto_cmd = Task.NO_TASK

    def switch_mode(self, do=1):
        if not do:
            return
        self.hd_sub_mode = (self.hd_sub_mode + 1) % len(HDSubMode)
        msg = Int8(data=self.hd_sub_mode)
        self.sub_mode_change_pub.publish(msg)

    def set_manual_velocity(self, joint_index, value, multiplier=1):
        #if self.hd_mode != HDMode.MANUAL_DIRECT: return
        if self.hd_sub_mode != HDSubMode.JOINTSPACE: return
        self.vel_cmd[joint_index] = value * multiplier
    
    def flip_manual_velocity_dir(self, joint_index, value):
        #if self.hd_mode != HDMode.MANUAL_DIRECT: return
        if self.hd_sub_mode != HDSubMode.JOINTSPACE: return
        if joint_index == 2:
            self.joint3_dir *= -1
        elif joint_index == 3:
            self.joint4_dir *= -1
    
    def set_gripper_speed(self, value, event_value):
        #if self.hd_mode != HDMode.MANUAL_DIRECT: return
        #if self.hd_sub_mode != HDSubMode.JOINTSPACE: return
        self.vel_cmd[6] = value * event_value
        self.torque_scaling_factor = value * event_value
    
    def set_rassor_speed(self, value, event_value):
        #if self.hd_mode != HDMode.MANUAL_DIRECT: return
        if self.hd_sub_mode != HDSubMode.JOINTSPACE: return
        self.vel_cmd[7] = value * event_value
    
    def set_man_inv_axis(self, coordinate, value, multiplier=1):
        #if self.hd_mode != HDMode.MANUAL_INVERSE: return
        if self.hd_sub_mode != HDSubMode.CARTESIAN: return
        #self.man_inv_axis[coordinate] = 0.0 if value < 0.5 else 1.0 * multiplier
        v = clean(value * multiplier)
        if coordinate == 0:
            self.man_inv_twist.linear.x = v
        elif coordinate == 1:
            self.man_inv_twist.linear.y = v
        else:
            self.man_inv_twist.linear.z = v
        
    def set_man_inv_angular(self, coordinate, value, multiplier=1):
        #if self.hd_mode != HDMode.MANUAL_INVERSE: return
        if self.hd_sub_mode != HDSubMode.CARTESIAN: return
        v = clean(value * multiplier)
        if coordinate == 0:
            self.man_inv_twist.angular.x = v
        elif coordinate == 1:
            self.man_inv_twist.angular.y = v
        else:
            self.man_inv_twist.angular.z = v
    
    def set_semi_auto_cmd(self, index, event_value):
        if self.hd_mode != HDMode.SEMI_AUTONOMOUS: return
        if event_value != 1: return
        self.semi_auto_cmd = SemiAutoTask[index]
    
    def set_man_inv_velocity_scaling(self, value):
        return
        if self.hd_mode != HDMode.MANUAL_INVERSE: return
        self.man_inv_velocity_scaling = 1.0 - abs(value)


def main():
    rclpy.init()
    node = ControlStation()

    # Spin in a separate thread
    rclpy.spin(node)
    # thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    # thread.start()

    # try:
    #     node.read_gamepad()
    # except KeyboardInterrupt:
    #     pass

    # rclpy.shutdown()
    # thread.join()


if __name__ == '__main__':
    main()
