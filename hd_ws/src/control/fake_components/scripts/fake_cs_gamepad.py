#!/usr/bin/env python3

from typing import Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import threading
from time import sleep
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8, Bool
from kerby_interfaces.msg import Task
import math
import itertools


def clean(x):
    epsilon = 0.09
    return 0 if abs(x) < epsilon else x


def str_pad(x, length=10):
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length-len(s))


def add(l1: list, l2: list):
    return [x+y for x, y in zip(l1, l2)]


def normalized(l):
    n = math.sqrt(sum(x**2 for x in l))
    if n == 0:
        return l
    return [x/n for x in l]


class Inft_Timer():
    def __init__(self, t, target):
        self.t = t
        self.target = target
        self.thread = threading.Timer(self.t, self.handler)
    
    def handler(self):
        self.target()
        self.thread = threading.Timer(self.t, self.handler)
        self.thread.start()

    def start(self):
        self.thread = threading.Timer(self.t, self.handler)
        self.thread.start()

    def cancel(self):
        self.thread.cancel()


class GamePadConfig:
    def __init__(self, joint_event_codes, joint3_retreat_code, joint4_retreat_code, joint_event_offsets, joint_event_amplitudes, gripper_event_codes, mode_switch_code, rassor_event_codes):
        self.joint_event_codes = joint_event_codes
        self.joint3_retreat_code = joint3_retreat_code
        self.joint4_retreat_code = joint4_retreat_code
        self.joint_event_offsets = joint_event_offsets
        self.joint_event_amplitudes = joint_event_amplitudes    # amplitude in one direction (so half the total amplitude) (allowing for negative amplitudes which effectively reverses the direction of the joint)
        self.gripper_event_codes = gripper_event_codes  # corresponding to respectively circle, triangle, square, cross
        self.rassor_event_codes = rassor_event_codes
        self.mode_switch_code = mode_switch_code


class EventFuncWrapper:
    def __init__(self, func=None, event_value_arg_name="event_value", **kwargs):
        if func is None:
            func = lambda *args, **kwargs: None
        self.func = func
        self.event_value_arg_name = event_value_arg_name
        self.kwargs = kwargs
        self.kwargs[event_value_arg_name] = None
    
    def __call__(self, event_value):
        self.kwargs[self.event_value_arg_name] = event_value
        return self.func(**self.kwargs)

        
class GamePadConfig2:
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
        # for _, name in self.BUTTONS.items():
        #     attr = f"{name}_id"
        #     setattr(self, attr, kwargs.get(attr, 0))
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

    def bind(self, input, func, event_value_arg_name, **kwargs):
        if input in self.INPUTS:
            self.event_signals[input].append(EventFuncWrapper(func, event_value_arg_name, **kwargs))
    
    def handle_event(self, event):
        if event.type == 0:
            return      # ignore this type of event (not sure what it corresponds to but results in weird behaviour)
        for input, id in self.input_ids.items():
            if event.code == id:
                self.emmit_signal(input, event.value)
    
    def emmit_signal(self, input, raw_value):
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




DEFAULT_CONFIG = GamePadConfig(
    joint_event_codes = [3, 4, 5, 2, 1, 0],
    joint3_retreat_code = 311,
    joint4_retreat_code = 310,
    joint_event_offsets = [0, 0, 0, 0, 0, 0],
    joint_event_amplitudes = [2**15, 2**15, 2**8, 2**8, 2**15, 2**15],    # amplitude in one direction (so half the total amplitude)
    gripper_event_codes = [305, 308, 307, 304],      # giving respectively 1, 0.1, -1, -0.1
    rassor_event_codes = [16, 17],
    mode_switch_code = 316
)

KNOWN_CONFIGS = {}
KNOWN_CONFIGS["Wireless Controller"] = GamePadConfig(
    joint_event_codes = [3, 4, 5, 2, 1, 0],
    joint3_retreat_code = 311,
    joint4_retreat_code = 310,
    joint_event_offsets = [2**7, 2**7, 0, 0, 2**7, 2**7],
    joint_event_amplitudes = [-2**7, 2**7, 2**8, 2**8, 2**7, 2**7],
    gripper_event_codes = [305, 307, 308, 304],
    rassor_event_codes = [16, 17],
    mode_switch_code = 316
)

KNOWN_CONFIGS["Generic X-Box pad"] = GamePadConfig(
    joint_event_codes = [3, 4, 5, 2, 1, 0],
    joint3_retreat_code = 311,
    joint4_retreat_code = 310,
    joint_event_offsets = [0, 0, 0, 0, 0, 0],
    joint_event_amplitudes = [-2**15, 2**15, 2**8, 2**8, 2**15, 2**15],
    gripper_event_codes = [305, 308, 307, 304],
    rassor_event_codes = [16, 17],
    mode_switch_code = 316
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


HD_MODES = Enum(
    MANUAL_INVERSE = 0,
    MANUAL_DIRECT = 1,
    SEMI_AUTONOMOUS = 2,
    AUTONOMOUS = 3
)


SEMI_AUTO_TASKS = Enum(
    NO_TASK = Task.NO_TASK,
    BTN_TASK = Task.BUTTON,
    PLUG_VOLTMETER = Task.PLUG_VOLTMETER_APPROACH,
    NAMED_TARGET_TASK = Task.NAMED_TARGET
)


class GamePad(Node):
    MAX_VELOCITIES = [1, 1, 1, 1, 1, 1, 1, 1]

    def __init__(self):
        super().__init__("fake_cs_gamepad")

        self.vel_cmd = [0.0]*8
        self.axis_cmd = [0.0]*3
        self.man_inv_data = [0]*6
        self.man_inv_vectors = [    # possible vectors for manual inverse translation (actually the resulting vector could be an integral linear combination of those)
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
        ]
        self.man_inv_velocity_scaling = 0.0
        self.semi_auto_cmd_id = -1      # -1 for no command
        self.semi_auto_cmd = Task.NO_TASK

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = HD_MODES.MANUAL_DIRECT

        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        #self.task_id_pub = self.create_publisher(Int8, "/ROVER/element_id", 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        dt = 1/50
        self.timer = Inft_Timer(dt, self.publish_cmd)
        self.connect()

        self.config = None
        self.config2 = GamePadConfig2()
        self.identify_device()
        self.create_bindings()

    def create_bindings(self):
        for i, input in enumerate([GamePadConfig2.RH, GamePadConfig2.RV, GamePadConfig2.R2, GamePadConfig2.L2, GamePadConfig2.LV, GamePadConfig2.LH]):
            self.config2.bind(input, self.set_manual_velocity, "value", joint_index=i)

        self.config2.bind(GamePadConfig2.R1, self.flip_manual_velocity_dir, "value", joint_index=2)
        self.config2.bind(GamePadConfig2.L1, self.flip_manual_velocity_dir, "value", joint_index=3)
        self.config2.bind(GamePadConfig2.PS, self.switch_mode, "do")

        for val, input in zip([1.0, -1.0, 0.1, -0.1], [GamePadConfig2.CIRCLE, GamePadConfig2.SQUARE, GamePadConfig2.TRIANGLE, GamePadConfig2.CROSS]):
            self.config2.bind(input, self.set_gripper_speed, "event_value", value=val)

        self.config2.bind(GamePadConfig2.DIRH, self.set_rassor_speed, "event_value", value=1.0)
        self.config2.bind(GamePadConfig2.DIRV, self.set_rassor_speed, "event_value", value=-0.1)

        for i, input in enumerate([GamePadConfig2.CIRCLE, GamePadConfig2.SQUARE, GamePadConfig2.TRIANGLE, GamePadConfig2.CROSS, GamePadConfig2.R2, GamePadConfig2.L2]):
            self.config2.bind(input, self.set_man_inv_axis, "value", coordinate=i//2, multiplier=(-1)**i)

    def connect(self):
        print("connecting")
        self.device = None
        while not self.device:
            for device in map(evdev.InputDevice, evdev.list_devices()):
            #for device in [evdev.InputDevice(path) for path in evdev.list_devices()]: 
                print(device)
                self.device = device
                self.timer.start()
                return device
            sleep(1)
    
    def identify_device(self):
        name = self.device.name
        if name not in KNOWN_CONFIGS:
            self.config = DEFAULT_CONFIG
            return
        self.config = KNOWN_CONFIGS[name]
        self.config2 = GamePadConfig2.from_name(name)

    def publish_cmd(self):
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            l = [v*x for v,x in zip(self.MAX_VELOCITIES, list(map(clean, map(float, self.vel_cmd))))]
            l[2] *= self.joint3_dir
            l[3] *= self.joint4_dir
            print("[", ", ".join(map(str_pad, l)), "]")
            l = [1.0] + l   # add dummy velocity scaling factor
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data = l))
        elif self.hd_mode == HD_MODES.MANUAL_INVERSE:
            axis = [0.0, 0.0, 0.0]
            for vec, d in zip(self.man_inv_vectors, self.man_inv_data):
                if d:
                    axis = add(axis, vec)
            axis = normalized(axis)
            axis = normalized(self.man_inv_data[:3])
            print("[", ", ".join(map(str_pad, axis)), "]")
            data = [self.man_inv_velocity_scaling] + axis
            self.man_inv_axis_pub.publish(Float32MultiArray(data = data))
        elif self.hd_mode == HD_MODES.SEMI_AUTONOMOUS:
            if self.semi_auto_cmd == Task.NO_TASK:
                return
            elif self.semi_auto_cmd == Task.BUTTON:
                msg = Task(type=Task.BUTTON, id=self.semi_auto_cmd_id)
                self.task_pub.publish(msg)
            elif self.semi_auto_cmd == Task.PLUG_VOLTMETER_APPROACH:
                msg = Task(type=Task.PLUG_VOLTMETER_APPROACH)
                self.task_pub.publish(msg)
            elif self.semi_auto_cmd == Task.NAMED_TARGET:
                msg = Task(type=Task.NAMED_TARGET, str_slot="optimal_view")
                self.task_pub.publish(msg)
            self.semi_auto_cmd = Task.NO_TASK
    
    def switch_mode(self, do=1):
        if not do:
            return
        self.hd_mode = (self.hd_mode + 1) % len(HD_MODES)
        msg = Int8()
        msg.data = self.hd_mode
        self.mode_change_pub.publish(msg)

    def set_manual_velocity(self, joint_index, value):
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            self.vel_cmd[joint_index] = value
    
    def flip_manual_velocity_dir(self, joint_index, value):
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            if joint_index == 2:
                self.joint3_dir *= -1
            elif joint_index == 3:
                self.joint4_dir *= -1
    
    def set_gripper_speed(self, value, event_value):
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            self.vel_cmd[6] = value * event_value
    
    def set_rassor_speed(self, value, event_value):
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            self.vel_cmd[7] = value * event_value
    
    def set_man_inv_axis(self, coordinate, value, multiplier):
        if self.hd_mode == HD_MODES.MANUAL_INVERSE:
            self.man_inv_data[coordinate] = 0.0 if value < 0.5 else 1.0 * multiplier

    def handle_continuous_event(self, event):
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            for i, (code, offset, amplitude) in enumerate(zip(self.config.joint_event_codes, self.config.joint_event_offsets, self.config.joint_event_amplitudes)):
                if event.code == code:
                    self.vel_cmd[i] = (event.value - offset) / amplitude
                    return
        
        elif self.hd_mode == HD_MODES.MANUAL_INVERSE:
            if event.code == self.config.joint_event_codes[2]:
                self.man_inv_data[4] = 1 if (event.value - self.config.joint_event_offsets[2]) / self.config.joint_event_amplitudes[2] > 0.5 else 0
            if event.code == self.config.joint_event_codes[3]:
                self.man_inv_data[5] = 1 if (event.value - self.config.joint_event_offsets[3]) / self.config.joint_event_amplitudes[3] > 0.5 else 0
            if event.code == self.config.joint_event_codes[4]:
                x = (event.value - self.config.joint_event_offsets[4]) / self.config.joint_event_amplitudes[4]
                self.man_inv_velocity_scaling = 1.0 - abs(x)
    
    def handle_binary_event_direct_mode(self, event):
        if event.value != 0 and event.value != 1: return

        if event.value == 1:
            for code, val in zip(self.config.gripper_event_codes, [1.0, 0.1, -1.0, -0.1]):
                if event.code == code:
                    self.vel_cmd[6] = val
                    return
        elif event.value == 0:
            self.vel_cmd[6] = 0.0
            self.man_inv_data[5] = 1 if (event.value - self.config.joint_event_offsets[3]) / self.config.joint_event_amplitudes[3] > 0.5 else 0

        if event.code == self.config.joint3_retreat_code:
            self.joint3_dir = -1 if event.value == 1 else 1
        if event.code == self.config.joint4_retreat_code:
            self.joint4_dir = -1 if event.value == 1 else 1
    
    def handle_binary_event_manual_inverse_mode(self, event):
        for i, code in enumerate(self.config.gripper_event_codes):
            if event.code == code:
                self.man_inv_data[i] = event.value
    
    def handle_binary_event_semi_auto_mode(self, event):
        if event.value != 1:
            return
        for i, code in enumerate(self.config.gripper_event_codes):
            if event.code == code:
                self.semi_auto_cmd = SEMI_AUTO_TASKS[i]
                if self.semi_auto_cmd == SEMI_AUTO_TASKS.BTN_TASK:
                    self.semi_auto_cmd_id = 1   # arbitrary

    def handle_binary_event(self, event):
        if event.code == self.config.mode_switch_code:
            if event.value == 1:
                self.switch_mode()
            return
        
        if self.hd_mode == HD_MODES.MANUAL_DIRECT:
            self.handle_binary_event_direct_mode(event)
        elif self.hd_mode == HD_MODES.MANUAL_INVERSE:
            self.handle_binary_event_manual_inverse_mode(event)
        elif self.hd_mode == HD_MODES.SEMI_AUTONOMOUS:
            self.handle_binary_event_semi_auto_mode(event)
    
    def handle_rassor_event(self, event):
        if self.hd_mode != HD_MODES.MANUAL_DIRECT or event.value not in [-1, 0, 1]:
            return
        if event.code == self.config.rassor_event_codes[0]:
            self.vel_cmd[7] = event.value
        elif event.code == self.config.rassor_event_codes[1]:
            self.vel_cmd[7] = event.value * -0.1
    
    def read_gamepad(self):
        while rclpy.ok():
            try: 
                for event in self.device.read_loop():
                    self.config2.handle_event(event)
                    # if event.type == 3:
                    #     self.handle_continuous_event(event)
                    #     self.handle_rassor_event(event)

                    # if event.type == 1:
                    #     self.handle_binary_event(event)

            except (TypeError, IOError) as e:
                self.timer.cancel()
                self.connect()
                print(e)

def main():
    rclpy.init()
    node = GamePad()

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