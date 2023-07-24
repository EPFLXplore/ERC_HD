#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
from time import sleep
import threading


class GamePad(Node):
    def __init__(self):
        super().__init__("fake_cs_gamepad")

        self.connect()

    def connect(self):
        print("connecting")
        self.device = None
        while not self.device : 
            for device in [evdev.InputDevice(path) for path in evdev.list_devices()]: 
                print(device)
                self.device = device
                return device
            sleep(1)

    def read_gamepad(self):
        while rclpy.ok():
            try : 
                for event in self.device.read_loop():
                    if event.type == 1:
                        print(event)

            except (TypeError, IOError):
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
