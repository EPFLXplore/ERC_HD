#!/usr/bin/env python2

#cest lui

import rospy
from geometry_msgs.msg import Twist
import evdev
import threading
from time import sleep
from std_msgs.msg      import Int8MultiArray, Int8, Bool

max_vel = 100#0.1
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

class GamePad():
    def __init__(self):
        #---------------HANDLING DEVICE----------------
        self.zero_HD = [0, 0, 0, 0, 0, 0, 0]
        self.axe_HD_old = [0, 0, 0, 0, 0, 0, 0]
        self.axe_HD_new = [0, 0, 0, 0, 0, 0, 0]
        self.joint = 0  # joint 1 as default joint

        self.joint3 = 1
        self.joint4 = 1

        self.reset_arm_pos = False

        self.HD_Angles_pub    = rospy.Publisher('HD_joints', Int8MultiArray, queue_size=1)
        self.HD_Reset_arm_pub = rospy.Publisher('HD_reset_arm_pos', Bool, queue_size=1)
        self.HD_set_zero_pub = rospy.Publisher('HD_set_zero_arm_pos', Bool, queue_size=1)
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
        #print("send HD - angles:\nold:", self.axe_HD_old , ' new:', self.axe_HD_new)
        for k in range(len(self.axe_HD_new)):
            self.axe_HD_old[k] = self.axe_HD_new[k]

        print(self.axe_HD_new)
        self.HD_Angles_pub.publish(Int8MultiArray(data = list(map(int, self.axe_HD_new))))


    def read_gamepad(self) :
        while not rospy.is_shutdown():
            try : 
                for event in self.device.read_loop():
                    if event.type == 3 :
                        if event.code == 0 : # x_axis
                            self.axe_HD_new[5] = max_vel * (event.value- 128)/128.0
                        if event.code == 1 : # y_axis
                            self.axe_HD_new[4] = -max_vel * (128 - event.value)/128.0
                        if event.code == 2 : # y_axis R3
                            self.axe_HD_new[0] = -max_vel * (event.value - 128)/128.0
                        if event.code == 5 : # x_axis R3
                            self.axe_HD_new[1] = -max_vel * (event.value - 128)/128.0
                        if event.code == 3 : # L2
                            self.axe_HD_new[3] = self.joint4 * (max_vel * (event.value - 128)/256.0 + 50)
                        if event.code == 4 : # R2
                            #print(self.joint3)
                            self.axe_HD_new[2] = self.joint3 * (max_vel * (event.value - 128)/256.0 + 50)
                    #print(event)
                    if event.type == 1 :
                        #print(event.code)
                        if event.value == 1:
                            #-----------gripper------------
                            if event.code == 307:   # gripper = +100
                                self.axe_HD_new[6] = 100.00
                            elif event.code == 306:   # gripper = +1
                                self.axe_HD_new[6] = 10
                            elif event.code == 305:    # gripper = -100
                                self.axe_HD_new[6] = -100.00
                            elif event.code == 304:   # gripper = -1
                                self.axe_HD_new[6] = -10
                            #----------joint 3, 4 retreat-----------
                            elif event.code == 309:       # R1 - joint 3 retreat 
                                self.joint3 = -1
                            elif event.code == 308:       # L1 - joint 4 retreat
                                self.joint4 = -1
                        #-----------joint 3, 4 advance------------
                            elif event.code == 317:
                                self.reset_arm_pos = not(self.reset_arm_pos)
                                msg = Bool()
                                msg.data = self.reset_arm_pos
                                self.HD_Reset_arm_pub.publish(msg)
                            elif event.code == 316:
                                msg = Bool()
                                self.HD_set_zero_pub.publish(msg)
                        elif event.value == 0:
                            self.axe_HD_new[6] = 0
                            if event.code == 309:         # R1 - joint 3 retreat
                                self.joint3 = 1
                            elif event.code == 308:       # L1 - joint 4 retreat
                                self.joint4 = 1

            except (TypeError, IOError):
                self.timer.cancel()
                self.connect()

def gamepad_reader_node():
    rospy.init_node('gamepad_reader_node')
    print("2")
    gp = GamePad()
    print("3")
    try :
        print("1")
        gp.read_gamepad()
    except :
        print("error")
        gp.timer.cancel()




if __name__ == "__main__" : 
    print("qQQQQQqqq")
    gamepad_reader_node()