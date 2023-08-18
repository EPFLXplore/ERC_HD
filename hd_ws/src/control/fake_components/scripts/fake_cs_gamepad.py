#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import threading
from time import sleep
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8, Bool
import math


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


class GamePad(Node):
    MANUAL_INVERSE = 0
    MANUAL_DIRECT = 1
    SEMI_AUTONOMOUS = 2
    AUTONOMOUS = 3
    HD_MODES = [MANUAL_INVERSE, MANUAL_DIRECT, SEMI_AUTONOMOUS, AUTONOMOUS]

    MAX_VELOCITIES = [1, 1, 1, 1, 1, 1, 1, 0]

    # default config
    JOINT_EVENT_CODES = [3, 4, 5, 2, 1, 0]
    JOINT3_RETREAT_CODE = 311
    JOINT4_RETREAT_CODE = 310
    JOINT_EVENT_OFFSETS = [0, 0, 0, 0, 0, 0]
    JOINT_EVENT_AMPLITUDES = [2**15, 2**15, 2**8, 2**8, 2**15, 2**15]    # amplitude in one direction (so half the total amplitude)
    GRIPPER_EVENT_CODES = [305, 308, 307, 304]      # giving respectively 1, 0.1, -1, -0.1
    MODE_SWITCH_CODE = 316

    # other configs
    KNOWN_CONFIGS = {}
    KNOWN_CONFIGS["Wireless Controller"] = {
        "joint_event_codes": [3, 4, 5, 2, 1, 0],
        "joint3_retreat_code": 311,
        "joint4_retreat_code": 310,
        "joint_event_offsets": [2**7, 2**7, 0, 0, 2**7, 2**7],
        "joint_event_amplitudes": [-2**7, 2**7, 2**8, 2**8, 2**7, 2**7],
        "gripper_event_codes": [305, 307, 308, 304],
        "mode_switch_code": 316
    }
    KNOWN_CONFIGS["Generic X-Box pad"] = {
        "joint_event_codes": [3, 4, 5, 2, 1, 0],
        "joint3_retreat_code": 311,
        "joint4_retreat_code": 310,
        "joint_event_offsets": [0, 0, 0, 0, 0, 0],
        "joint_event_amplitudes": [-2**15, 2**15, 2**8, 2**8, 2**15, 2**15],
        "gripper_event_codes": [305, 308, 307, 304],
        "mode_switch_code": 316
    }

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
        self.semi_auto_cmd_id = -1      # -1 for no command

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = self.MANUAL_DIRECT

        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_gamepad", 10)
        self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.task_id_pub = self.create_publisher(Int8, "/ROVER/element_id", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        dt = 1/50
        self.timer = Inft_Timer(dt, self.publish_cmd)
        self.connect()
        self.identify_device()

    def connect(self):
        print("connecting")
        self.device = None
        while not self.device : 
            for device in [evdev.InputDevice(path) for path in evdev.list_devices()] : 
                print(device)
                self.device = device
                self.timer.start()
                return device
            sleep(1)
    
    def identify_device(self):        
        name = self.device.name
        if name not in self.KNOWN_CONFIGS:
            return
        config = self.KNOWN_CONFIGS[name]
        self.JOINT_EVENT_CODES = config["joint_event_codes"]
        self.JOINT3_RETREAT_CODE = config["joint3_retreat_code"]
        self.JOINT4_RETREAT_CODE = config["joint4_retreat_code"]
        self.JOINT_EVENT_OFFSETS = config["joint_event_offsets"]
        self.JOINT_EVENT_AMPLITUDES = config["joint_event_amplitudes"]
        self.GRIPPER_EVENT_CODES = config["gripper_event_codes"]
        self.MODE_SWITCH_CODE = config["mode_switch_code"]

    def publish_cmd(self):
        if self.hd_mode == self.MANUAL_DIRECT:
            l = [v*x for v,x  in zip(self.MAX_VELOCITIES, list(map(clean, map(float, self.vel_cmd))))]
            l[2] *= self.joint3_dir
            l[3] *= self.joint4_dir
            print("[", ", ".join(map(str_pad, l)), "]")
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data = l))
        elif self.hd_mode == self.MANUAL_INVERSE:
            axis = [0.0, 0.0, 0.0]
            for vec, d in zip(self.man_inv_vectors, self.man_inv_data):
                if d:
                    axis = add(axis, vec)
            axis = normalized(axis)
            print("[", ", ".join(map(str_pad, axis)), "]")
            self.man_inv_axis_pub.publish(Float32MultiArray(data = axis))
        elif self.hd_mode == self.SEMI_AUTONOMOUS:
            if self.semi_auto_cmd_id == -1: 
                return
            msg = Int8(data = self.semi_auto_cmd_id)
            self.task_id_pub.publish(msg)
            self.semi_auto_cmd_id = -1
    
    def switch_mode(self):
        self.hd_mode = (self.hd_mode + 1) % len(self.HD_MODES)
        msg = Int8()
        msg.data = self.hd_mode
        self.mode_change_pub.publish(msg)

    def handle_continuous_event(self, event):
        if self.hd_mode == self.MANUAL_DIRECT:
            for i, (code, offset, amplitude) in enumerate(zip(self.JOINT_EVENT_CODES, self.JOINT_EVENT_OFFSETS, self.JOINT_EVENT_AMPLITUDES)):
                if event.code == code:
                    self.vel_cmd[i] = (event.value - offset) / amplitude
                    return
        
        elif self.hd_mode == self.MANUAL_INVERSE:
            if event.code == self.JOINT_EVENT_CODES[2]:
                self.man_inv_data[4] = 1 if (event.value - self.JOINT_EVENT_OFFSETS[2]) / self.JOINT_EVENT_AMPLITUDES[2] > 0.5 else 0
            if event.code == self.JOINT_EVENT_CODES[3]:
                self.man_inv_data[5] = 1 if (event.value - self.JOINT_EVENT_OFFSETS[3]) / self.JOINT_EVENT_AMPLITUDES[3] > 0.5 else 0
    
    def handle_binary_event_direct_mode(self, event):
        if event.value != 0 and event.value != 1: return

        if event.value == 1:
            for code, val in zip(self.GRIPPER_EVENT_CODES, [1.0, 0.1, -1.0, -0.1]):
                if event.code == code:
                    self.vel_cmd[6] = val
                    return
        elif event.value == 0:
            self.vel_cmd[6] = 0.0
            self.man_inv_data[5] = 1 if (event.value - self.JOINT_EVENT_OFFSETS[3]) / self.JOINT_EVENT_AMPLITUDES[3] > 0.5 else 0

        if event.code == self.JOINT3_RETREAT_CODE:
            self.joint3_dir = -1 if event.value == 1 else 1
        if event.code == self.JOINT4_RETREAT_CODE:
            self.joint4_dir = -1 if event.value == 1 else 1
    
    def handle_binary_event_manual_inverse_mode(self, event):
        for i, code in enumerate(self.GRIPPER_EVENT_CODES):
            if event.code == code:
                self.man_inv_data[i] = event.value
    
    def handle_binary_event_semi_auto_mode(self, event):
        if event.value != 1:
            return
        for i, code in enumerate(self.GRIPPER_EVENT_CODES):
            if event.code == code:
                self.semi_auto_cmd_id = i

    def handle_binary_event(self, event):
        if event.code == self.MODE_SWITCH_CODE:
            if event.value == 1:
                self.switch_mode()
            return
        
        if self.hd_mode == self.MANUAL_DIRECT:
            self.handle_binary_event_direct_mode(event)
        elif self.hd_mode == self.MANUAL_INVERSE:
            self.handle_binary_event_manual_inverse_mode(event)
        elif self.hd_mode == self.SEMI_AUTONOMOUS:
            self.handle_binary_event_semi_auto_mode(event)

    def read_gamepad(self):
        while rclpy.ok():
            try: 
                for event in self.device.read_loop():
                    if event.type == 3:
                        self.handle_continuous_event(event)

                    if event.type == 1:
                        self.handle_binary_event(event)

            except (TypeError, IOError):
                self.timer.cancel()
                self.connect()

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