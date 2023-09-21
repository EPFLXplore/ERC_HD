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
from kerby_interfaces.msg import Task
import math
import itertools
from collections.abc import Callable
from typing import List, Any, Union


def clean(x: float) -> float:
    epsilon = 0.09
    return 0 if abs(x) < epsilon else x


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


class EventFuncWrapper:
    def __init__(self, func: Callable =None, event_value_arg_name="event_value", **kwargs):
        if func is None:
            func = lambda *args, **kwargs: None
        self.func = func
        self.event_value_arg_name = event_value_arg_name
        self.kwargs = kwargs
        self.kwargs[event_value_arg_name] = None
    
    def __call__(self, event_value) -> Any:
        self.kwargs[self.event_value_arg_name] = event_value
        return self.func(**self.kwargs)

        
class GamePadConfig:
    # TODO: create an interface for the touchpad
    TRIANGLE = 0
    SQUARE = 1
    CIRCLE = 2
    CROSS = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    L3 = 8
    R3 = 9
    CREATE = 10
    OPTIONS = 11
    PS = 12
    TOUCHPAD = 13
    LH = 14
    LV = 15
    RH = 16
    RV = 17
    DIRH = 18
    DIRV = 19
    
    BUTTONS = {TRIANGLE : "triangle", SQUARE : "square", CIRCLE: "circle", CROSS: "cross", L1: "L1", R1: "R1", L3: "L3", R3: "R3", CREATE: "create", OPTIONS: "options", PS: "PS", TOUCHPAD: "touchpad"}
    ANALOG_TRIGGERS = {RH: "RH", RV: "RV", LH: "LH", LV: "LV", L2: "L2", R2: "R2", DIRH: "dirH", DIRV: "dirV"}
    INPUTS = BUTTONS | ANALOG_TRIGGERS
    DEFAULT_IDS = {inp: -1 for inp in INPUTS}
    DEFAULT_IDS |= {L1: 310, R1: 311, L2: 2, R2: 5, PS: 316, LH: 0, LV: 1, RH: 3, RV: 4, DIRH: 16, DIRV: 17, TRIANGLE: 307, SQUARE: 308, CIRCLE: 305, CROSS: 304}

    # simple buttons
    BUTTON_NAMES = ["triangle", "square", "circle", "cross", "L1", "R1", "L3", "R3", "create", "options", "PS", "touchpad"]
    # joysticks and analog buttons
    ANALOG_TRIGGER_NAMES = ["RH", "RV", "LH", "LV", "L2", "R2", "dirH", "dirV"]   # L, R for left, right and H, V for horizontal, vertical

    def __init__(self, **kwargs):
        self.input_ids = {}
        self.input_offsets = {}
        self.input_amplitudes = {}
        for input, name in self.INPUTS.items():
            attr = f"{name}_id"
            self.input_ids[input] = kwargs.get(attr, self.DEFAULT_IDS[input])
            attr = f"{name}_offset"
            self.input_offsets[input] = kwargs.get(attr, 0.0)
            attr = f"{name}_amplitude"
            self.input_amplitudes[input] = kwargs.get(attr, 1.0)

            attr = f"{name}_id"
            setattr(self, attr, kwargs.get(attr, -1.0))
            attr = f"{name}_offset"
            # set offset and amplitude such that the values of the joystick go from -amplitude+offset to amplitude+offset or from offset to amplitude+offset depending on the trigger type
            setattr(self, attr, kwargs.get(attr, 0.0))
            attr = f"{name}_amplitude"
            setattr(self, attr, kwargs.get(attr, 1.0))

        self.event_signals = {k: [] for k in self.INPUTS}

    def bind(self, input, func: Callable, event_value_arg_name: str, **kwargs) -> None:
        if input in self.INPUTS:
            self.event_signals[input].append(EventFuncWrapper(func, event_value_arg_name, **kwargs))
    
    def handle_event(self, event: evdev.events.InputEvent) -> None:
        if event.type == 0:
            return      # ignore this type of event (not sure what it corresponds to but results in weird behaviour)
        for input, id in self.input_ids.items():
            if event.code == id:
                self.emmit_signal(input, event.value)
    
    def emmit_signal(self, input, raw_value: float):
        offset = self.input_offsets[input]
        amplitude = self.input_amplitudes[input]
        event_value = (raw_value - offset) / amplitude
        for func in self.event_signals[input]:
            func(event_value)

    @classmethod
    def from_name(cls, name):
        if name == "Generic X-Box pad":
            return cls(
                circle_id = 305,
                triangle_id = 308,
                square_id = 307,
                cross_id = 304,
                RH_amplitude = 2**15,
                RV_amplitude = 2**15,
                LH_amplitude = 2**15,
                LV_amplitude = 2**15,
                R2_amplitude = 2**8,
                L2_amplitude = 2**8,
            )
        elif name == "Wireless Controller":
            return cls(
                circle_id = 305,
                triangle_id = 307,
                square_id = 308,
                cross_id = 304,
                RH_offset = 2**7,
                RV_offset = 2**7,
                LH_offset = 2**7,
                LV_offset = 2**7,
                RH_amplitude = 2**7,
                RV_amplitude = 2**7,
                LH_amplitude = 2**7,
                LV_amplitude = 2**7,
                R2_amplitude = 2**8,
                L2_amplitude = 2**8,
            )


class Enum:
    # kinda overkill but for fun to mimic a C-style enum + some extra functionalities

    def __init__(self, **kwargs):
        self.items = kwargs
        self.slots = list(kwargs)
        print(self.slots)
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


HD_MODE = Enum(
    MANUAL_INVERSE = 0,
    MANUAL_DIRECT = 1,
    SEMI_AUTONOMOUS = 2,
    #AUTONOMOUS = 3
)


SEMI_AUTO_TASK = Enum(
    NO_TASK = Task.NO_TASK,
    BTN_TASK = Task.BUTTON,
    PLUG_VOLTMETER = Task.PLUG_VOLTMETER_APPROACH,
    NAMED_TARGET_TASK = Task.NAMED_TARGET
)


class ControlStation(Node):
    def __init__(self):
        super().__init__("fake_cs_gamepad")

        self.vel_cmd = [0.0]*8
        self.axis_cmd = [0.0]*3
        self.man_inv_axis = [0.0]*3
        self.man_inv_velocity_scaling = 1.0
        self.semi_auto_cmd = Task.NO_TASK

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = HD_MODE.MANUAL_DIRECT

        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        self.timer_period = 1/30
        #self.timer = Inft_Timer(dt, self.publish_cmd)
        self.timer2 = None
        self.gamepad = None
        self.connect()

        self.gamepad_config = GamePadConfig()
        self.identify_device()
        self.create_bindings()

    def create_bindings(self):
        for i, input in enumerate([GamePadConfig.RH, GamePadConfig.RV, GamePadConfig.R2, GamePadConfig.L2, GamePadConfig.LV, GamePadConfig.LH]):
            self.gamepad_config.bind(input, self.set_manual_velocity, "value", joint_index=i)

        self.gamepad_config.bind(GamePadConfig.R1, self.flip_manual_velocity_dir, "value", joint_index=2)
        self.gamepad_config.bind(GamePadConfig.L1, self.flip_manual_velocity_dir, "value", joint_index=3)
        self.gamepad_config.bind(GamePadConfig.PS, self.switch_mode, "do")

        for val, input in zip([1.0, -1.0, 0.1, -0.1], [GamePadConfig.CIRCLE, GamePadConfig.SQUARE, GamePadConfig.TRIANGLE, GamePadConfig.CROSS]):
            self.gamepad_config.bind(input, self.set_gripper_speed, "event_value", value=val)

        self.gamepad_config.bind(GamePadConfig.DIRH, self.set_rassor_speed, "event_value", value=1.0)
        self.gamepad_config.bind(GamePadConfig.DIRV, self.set_rassor_speed, "event_value", value=-0.1)

        for i, input in enumerate([GamePadConfig.L2, GamePadConfig.R2, GamePadConfig.CIRCLE, GamePadConfig.SQUARE, GamePadConfig.TRIANGLE, GamePadConfig.CROSS]):
            self.gamepad_config.bind(input, self.set_man_inv_axis, "value", coordinate=i//2, multiplier=(-1)**i)
            self.gamepad_config.bind(input, self.set_semi_auto_cmd, "event_value", index=i)
        
        self.gamepad_config.bind(GamePadConfig.LV, self.set_man_inv_velocity_scaling, "value")

    def connect(self):
        print("connecting")
        while self.gamepad is None:
            for device in map(evdev.InputDevice, evdev.list_devices()):
                print(device)
                self.gamepad = device
                #self.timer.start()
                self.timer2 = self.create_timer(self.timer_period, self.publish_cmd)
                return device
            sleep(1)

    def identify_device(self):
        name = self.gamepad.name
        self.gamepad_config = GamePadConfig.from_name(name)

    def publish_cmd(self):
        if self.hd_mode == HD_MODE.MANUAL_DIRECT:
            l = list(map(clean, map(float, self.vel_cmd)))
            l[2] *= self.joint3_dir
            l[3] *= self.joint4_dir
            print("[", ", ".join(map(str_pad, l)), "]")
            l = [1.0] + l   # add dummy velocity scaling factor
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data = l))
        elif self.hd_mode == HD_MODE.MANUAL_INVERSE:
            axis = normalized(self.man_inv_axis[:3])
            print("[", ", ".join(map(str_pad, axis)), "]")
            data = [self.man_inv_velocity_scaling] + axis
            self.man_inv_axis_pub.publish(Float32MultiArray(data = data))
        elif self.hd_mode == HD_MODE.SEMI_AUTONOMOUS:
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
        self.hd_mode = (self.hd_mode + 1) % len(HD_MODE)
        msg = Int8()
        msg.data = self.hd_mode
        self.mode_change_pub.publish(msg)

    def set_manual_velocity(self, joint_index, value):
        if self.hd_mode != HD_MODE.MANUAL_DIRECT: return
        self.vel_cmd[joint_index] = value
    
    def flip_manual_velocity_dir(self, joint_index, value):
        if self.hd_mode != HD_MODE.MANUAL_DIRECT: return
        if joint_index == 2:
            self.joint3_dir *= -1
        elif joint_index == 3:
            self.joint4_dir *= -1
    
    def set_gripper_speed(self, value, event_value):
        if self.hd_mode != HD_MODE.MANUAL_DIRECT: return
        self.vel_cmd[6] = value * event_value
    
    def set_rassor_speed(self, value, event_value):
        if self.hd_mode != HD_MODE.MANUAL_DIRECT: return
        self.vel_cmd[7] = value * event_value
    
    def set_man_inv_axis(self, coordinate, value, multiplier):
        if self.hd_mode != HD_MODE.MANUAL_INVERSE: return
        self.man_inv_axis[coordinate] = 0.0 if value < 0.5 else 1.0 * multiplier
    
    def set_semi_auto_cmd(self, index, event_value):
        if self.hd_mode != HD_MODE.SEMI_AUTONOMOUS: return
        if event_value != 1: return
        self.semi_auto_cmd = SEMI_AUTO_TASK[index]
    
    def set_man_inv_velocity_scaling(self, value):
        if self.hd_mode != HD_MODE.MANUAL_INVERSE: return
        self.man_inv_velocity_scaling = 1.0 - abs(value)

    def read_gamepad(self):
        while rclpy.ok():
            try:
                for event in self.gamepad.read_loop():
                    self.gamepad_config.handle_event(event)

            except (TypeError, IOError) as e:
                #self.timer.cancel()
                self.timer2.cancel()
                self.connect()
                print(e)

def main():
    rclpy.init()
    node = ControlStation()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    try:
        node.read_gamepad()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
