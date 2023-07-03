#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import threading
from time import sleep
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8, Bool


def clean(x):
    return 0 if abs(x) < 0.05 else x


def str_pad(x, length=10):
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length-len(s))


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

    MAX_VELOCITIES = [1, 1, 1, 1, 1, 1, 1, 1]

    # default config
    JOINT_EVENT_CODES = [3, 4, 5, 2, 1, 0]
    JOINT3_RETREAT_CODE = 311
    JOINT4_RETREAT_CODE = 310
    JOINT_EVENT_OFFSETS = [0, 0, 0, 0, 0, 0]
    JOINT_EVENT_AMPLITUDES = [2**15, 2**15, 2**8, 2**8, 2**15, 2**15]    # amplitude in one direction (so half the total amplitude)
    JOINT_EVENT_OFFSETS = [2**7, 2**7, 0, 0, 2**7, 2**7]
    JOINT_EVENT_AMPLITUDES = [2**7, 2**7, 2**8, 2**8, 2**7, 2**7]
    GRIPPER_EVENT_CODES = [305, 308, 307, 304]      # giving respectively 1, 0.1, -1, -0.1
    MODE_SWITCH_CODE = 316

    # other configs
    KNOWN_CONFIGS = {}
    KNOWN_CONFIGS["Wireless Controller"] = {
        "joint_event_codes": [3, 4, 5, 2, 1, 0],
        "joint3_retreat_code": 311,
        "joint4_retreat_code": 310,
        "joint_event_offsets": [2**7, 2**7, 0, 0, 2**7, 2**7],
        "joint_event_amplitutes": [2**7, 2**7, 2**8, 2**8, 2**7, 2**7],
        "gripper_event_codes": [305, 308, 307, 304],
        "mode_switch_code": 316
    }

    def __init__(self):
        super().__init__("fake_cs_gamepad")

        self.vel_cmd = [0.0]*8

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = self.MANUAL_DIRECT

        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_gamepad", 10)
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
        self.JOINT_EVENT_AMPLITUDES = config["joint_event_amplitutes"]
        self.GRIPPER_EVENT_CODES = config["gripper_event_codes"]
        self.MODE_SWITCH_CODE = config["mode_switch_code"]

    def publish_cmd(self):
        l = [v*x for v,x  in zip(self.MAX_VELOCITIES, list(map(clean, map(float, self.vel_cmd))))]
        print("[", ", ".join(map(str_pad, l)), "]")
        self.joint_vel_cmd_pub.publish(Float32MultiArray(data = l))

    def read_gamepad(self) :
        while rclpy.ok():
            try : 
                for event in self.device.read_loop():
                    if event.type == 3:
                        if event.code == self.JOINT_EVENT_CODES[0]:   # J1
                            self.vel_cmd[0] = -(event.value - self.JOINT_EVENT_OFFSETS[0]) / self.JOINT_EVENT_AMPLITUDES[0]
                        if event.code == self.JOINT_EVENT_CODES[1]:   # J2
                            self.vel_cmd[1] = (event.value - self.JOINT_EVENT_OFFSETS[1]) / self.JOINT_EVENT_AMPLITUDES[1]
                        if event.code == self.JOINT_EVENT_CODES[2]:   # J3
                            self.vel_cmd[2] = self.joint3_dir * (event.value - self.JOINT_EVENT_OFFSETS[2]) / self.JOINT_EVENT_AMPLITUDES[2]
                        if event.code == self.JOINT_EVENT_CODES[3]:   # J4
                            self.vel_cmd[3] = self.joint4_dir * (event.value - self.JOINT_EVENT_OFFSETS[3]) / self.JOINT_EVENT_AMPLITUDES[3]
                        if event.code == self.JOINT_EVENT_CODES[4]:   # J5
                            self.vel_cmd[4] = (event.value - self.JOINT_EVENT_OFFSETS[4]) / self.JOINT_EVENT_AMPLITUDES[4]
                        if event.code == self.JOINT_EVENT_CODES[5]:   # J6
                            self.vel_cmd[5] = (event.value - self.JOINT_EVENT_OFFSETS[5]) / self.JOINT_EVENT_AMPLITUDES[5]

                    if event.type == 1:
                        if event.value == 1:
                            #-----------gripper------------
                            if event.code == self.GRIPPER_EVENT_CODES[0]:
                                self.vel_cmd[6] = 1.0
                            elif event.code == self.GRIPPER_EVENT_CODES[1]:
                                self.vel_cmd[6] = 0.1
                            elif event.code == self.GRIPPER_EVENT_CODES[2]:
                                self.vel_cmd[6] = -1.0
                            elif event.code == self.GRIPPER_EVENT_CODES[3]:
                                self.vel_cmd[6] = -0.1
                            #----------joint 3, 4 retreat-----------
                            elif event.code == 311:       # joint 3 retreat 
                                self.joint3_dir = -1
                            elif event.code == 310:       # joint 4 retreat
                                self.joint4_dir = -1
                            #----------mode change----------
                            elif event.code == self.MODE_SWITCH_CODE:
                                self.hd_mode = (self.hd_mode + 1) % len(self.HD_MODES)
                                msg = Int8()
                                msg.data = self.hd_mode
                                self.mode_change_pub.publish(msg)

                        elif event.value == 0:
                            self.vel_cmd[6] = 0
                            if event.code == self.JOINT3_RETREAT_CODE:         # joint 3 stop retreat
                                self.joint3_dir = 1
                            elif event.code == self.JOINT4_RETREAT_CODE:       # joint 4 stop retreat
                                self.joint4_dir = 1

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
