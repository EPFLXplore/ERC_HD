#!/usr/bin/env python

import sys
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8, Bool
import keyboard


class Tracker:
    def __init__(self, filename):
        rospy.Subscriber("/arm_control/manual_cmd", Float32MultiArray, self.cmd_callback)
        rospy.Subscriber("/arm_tracking", Bool, self.tracking_callback)
        rospy.Subscriber("/save_tracking", Bool, self.write_to_file)
        self.start_timestamp = 0
        self.write_timestamp = 0
        self.commands = []
        self.tracking = False
        self.filename = filename

    def cmd_callback(self, msg):
        if not self.tracking: return
        timestamp = time.time() - self.start_timestamp
        self.commands.append((timestamp, msg.data))

    def tracking_callback(self, msg):
        if self.tracking:
            self.stop_tracking()
        else:
            self.start_tracking()

    def start_tracking(self):
        self.start_timestamp = time.time()
        self.tracking = True
        self.commands = []
    
    def stop_tracking(self):
        self.tracking = False
        timestamp = time.time() - self.start_timestamp
        self.commands.append((timestamp, [0]*7))
        self.start_timestamp = time.time()
    
    def write_to_file(self):
        self.write_timestamp = time.time()
        with open(self.filename, "w") as f:
            for t, cmd in self.commands:
                f.write(str(t) + "," + str(list(cmd)) + "\n")
        print("written " + str(len(self.commands)) + " commands to file " + self.filename)

    def loop(self):
        rospy.init_node("tracker")
        treshold = 2    # s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if keyboard.is_pressed("s") and time.time()-self.start_timestamp > treshold:
                if self.tracking:
                    print("stop recording")
                    self.stop_tracking()
                else:
                    print("start recording")
                    self.start_tracking()
            if keyboard.is_pressed("w") and time.time()-self.write_timestamp > treshold:
                self.write_to_file()
            rate.sleep()


def main():
    filename = raw_input("enter file to write on : ")
    tracker = Tracker(filename)
    tracker.loop()


if __name__ == "__main__":
    main()