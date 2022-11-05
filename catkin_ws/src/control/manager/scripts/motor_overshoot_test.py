#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8
from sensor_msgs.msg import JointState


FILENAME = "res22"
MOTOR_COUNT = 1
LIMITS = [0, 1]
MAX_SPEED = 10.0

PLAN = [10]*10 + [8]*10 + [6]*10 + [5]*10 + [4]*10 + [2]*10 + [0.5]*10

class Executor:
    def __init__(self):
        self.direction = 1
        self.motor_state = [0]*MOTOR_COUNT
        self.vel = 0

        self.real_pos = [0]*MOTOR_COUNT

        self.angles_pub = rospy.Publisher('HD_angles', Int8MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher('HD_ManualVelocity', Float32, queue_size=10)
        
        self.limit_reached = True
        self.limit_reached_at = 0
        self.waiting_time = 1
        self.task = 0
        self.finished = False
        self.results = []
        self.file = FILENAME

    def telem_callback(self, msg):
        self.real_pos = msg.position

    def publish_state(self):
        msg = Int8MultiArray()
        msg.data = self.motor_state[:]
        self.angles_pub.publish(msg)

    def publish_vel(self):
        msg = Float32()
        msg.data = self.vel
        self.vel_pub.publish(msg)

    def waited_enough(self):
        return (time.time()-self.limit_reached_at) > self.waiting_time

    def new_task(self):
        if self.task == len(PLAN):
            self.finished = True
            return
        self.vel = min(MAX_SPEED, PLAN[self.task])/MAX_SPEED*self.direction
        self.motor_state[0] = 1
        self.task += 1
        print("task " + str(self.task) + ": " + str(PLAN[self.task-1]) + "rpm")

    def update_task(self):
        if not(self.limit_reached) and self.direction*self.real_pos[0] > self.direction*LIMITS[(self.direction+1)//2]:
            self.limit_reached = True
            self.limit_reached_at = time.time()
        elif self.limit_reached and self.waited_enough():
            self.results.append(self.real_pos[0]-LIMITS[(self.direction+1)//2])
            self.direction *= -1
            self.limit_reached = False
            self.limit_reached_at = 0
            self.new_task()

    def write_results_to_disk(self):
        with open(self.file, "w") as f:
            f.write("DIR 1 :\n")
            for i in range(1, len(PLAN)):
                f.write("\t")
                f.write(str(PLAN[i]) + "rpm : ")
                f.write(str(self.results[i+1]))
                f.write("\n")

    def main(self):
        rospy.init_node('CS_sim_node', anonymous=True)
        rospy.Subscriber("/arm_control/joint_telemetry", JointState, self.telem_callback)
        rate = rospy.Rate(10) # 10hz
        rospy.logwarn("CS_sim started")
        while not(rospy.is_shutdown()) and not(self.finished):
            self.update_task()
            self.publish_state()
            self.publish_vel()
            rate.sleep()
        self.write_results_to_disk()

if __name__ == '__main__':
    try:
        Executor().main()
    except rospy.ROSInterruptException:
        pass
