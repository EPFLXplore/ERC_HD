#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import threading
from time import sleep
from std_msgs.msg import Float64MultiArray, Int8, Bool

max_vel = 1
dt    = 1/50


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
    def __init__(self):
        super().__init__("fake_cs_gamepad")

        self.vel_cmd = [0, 0, 0, 0, 0, 0, 0]

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.HD_Angles_pub = self.create_publisher(Float64MultiArray, "/CS/vel_joint_cmd", 10)
        self.timer = Inft_Timer(dt, self.publish_twist)
        self.connect()

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

    def publish_twist(self) :
        print(self.vel_cmd)
        self.HD_Angles_pub.publish(Float64MultiArray(data = list(map(float, self.vel_cmd))))

    def read_gamepad(self) :
        while rclpy.ok():
            try : 
                for event in self.device.read_loop():
                    if event.type == 3:
                        if event.code == 3:
                            self.vel_cmd[0] = -max_vel * event.value / 2**15
                        if event.code == 4:
                            self.vel_cmd[1] = max_vel * event.value / 2**15
                        if event.code == 5:
                            self.vel_cmd[2] = self.joint3_dir * max_vel * event.value / 2**8
                        if event.code == 2:
                            self.vel_cmd[3] = self.joint4_dir * max_vel * event.value / 2**8
                        if event.code == 1:
                            self.vel_cmd[4] = max_vel * event.value / 2**15
                        if event.code == 0:
                            self.vel_cmd[5] = max_vel * event.value / 2**15

                    if event.type == 1:
                        if event.value == 1:
                            #-----------gripper------------
                            if event.code == 305:
                                self.vel_cmd[6] = 100.00
                            elif event.code == 308:
                                self.vel_cmd[6] = 10
                            elif event.code == 307:
                                self.vel_cmd[6] = -100.00
                            elif event.code == 304:
                                self.vel_cmd[6] = -10
                            #----------joint 3, 4 retreat-----------
                            elif event.code == 311:       # joint 3 retreat 
                                self.joint3_dir = -1
                            elif event.code == 310:       # joint 4 retreat
                                self.joint4_dir = -1

                        elif event.value == 0:
                            self.vel_cmd[6] = 0
                            if event.code == 311:         # joint 3 stop retreat
                                self.joint3_dir = 1
                            elif event.code == 310:       # joint 4 stop retreat
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
