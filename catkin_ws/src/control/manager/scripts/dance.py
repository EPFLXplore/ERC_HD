#!/usr/bin/env python

import sys
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8, Bool

MOTOR_COUNT = 7


class DanceMove:
    def __init__(self, velocities, lifespan):
        self.velocities = velocities
        self.lifespan = lifespan
    
    def execute(self, dancer):
        msg = Float32MultiArray()
        msg.data = self.velocities
        dancer.manual_cmd_pub.publish(msg)
        while dancer.running and time.time()-dancer.start_timestamp < self.lifespan:
            pass
    
    def __str__(self):
        return str(self.lifespan) + " " + str(self.velocities)


class Dancer:
    HOME = [0]*MOTOR_COUNT
    NULL_SPEED = Float32MultiArray()
    NULL_SPEED.data = [0]*MOTOR_COUNT

    def __init__(self):
        rospy.Subscriber("/arm_control/joint_telemetry", JointState, self.telemCallback)
        rospy.Subscriber("HD_engage_dance", Bool, self.actionCallback)
        self.reset_arm_pub = rospy.Publisher('/arm_control/reset_arm_pos', Bool, queue_size=1)
        self.manual_cmd_pub = rospy.Publisher('/arm_control/manual_cmd', Float32MultiArray, queue_size=10)

        self.actual_joint_pos = [0]*MOTOR_COUNT
        self.goal = None
        self.running = False
        self.dance_moves = []
        self.going_home = False
        self.start_timestamp = 0

    def read_from_file(self, filename):
        try:
            for line in open(filename):
                line = line.strip()
                i = line.index(",")
                lifespan = float(line[:i])
                vels = eval(line[i+1:])
                self.dance_moves.append(DanceMove(vels, lifespan))
        except:
            print("error in reading file")
            raise

    def telemCallback(self, msg):
        self.actual_joint_pos = msg.position
    
    def actionCallback(self, msg):
        if not self.running:
            self.running = True
            self.execute_moves()
        else:
            self.stop()

    def goal_far_away(self):
        acceptable_distance = [10**-4, 10**-4, 10**-1, 10**-4, 10**-4, 10**-4, 10**-4]
        return all(abs(self.actual_joint_pos[i]-self.goal[i]) < acceptable_distance[i] for i in range(MOTOR_COUNT))

    def wait_for_goal_achievement(self, additional_time=0):
        while self.goal_far_away():
            "vibe"
        time.sleep(additional_time)

    def goto_home(self, wait_time=2):
        self.going_home = True
        msg = Bool()
        msg.data = True
        self.reset_arm_pub.publish(msg)
        self.goal = self.HOME
        self.wait_for_goal_achievement(wait_time)
        msg.data = False
        self.reset_arm_pub.publish(msg)
        self.going_home = False
    
    def stop(self):
        self.running = False
        self.manual_cmd_pub.publish(self.NULL_SPEED)

    def execute_moves(self):
        print("going to home")
        #self.goto_home()
        print("reached home")
        self.start_timestamp = time.time()
        for move in self.dance_moves:
            if not self.running:
                break
            move.execute(self)
        self.stop()
        #self.goto_home()
     
    def dance(self):
        rospy.init_node("dancer")
        self.read_from_file("/home/xplore/hd-control/HD_workspace/catkin_ws/track")
        rospy.spin()
        
    def __str__(self):
        return "\n".join(map(str, self.dance_moves))


if __name__ == "__main__":
    dancer = Dancer()
    dancer.dance()
    #print(dancer)
