#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Float32MultiArray, Int8MultiArray, Float32, Int8, Bool, Int16
from task_execution.msg import Task


VERBOSE = True


class Manager:
    AUTONOMOUS = 0
    SEMI_AUTONOMOUS = 1
    MANUAL_INVERSE = 2
    MANUAL_DIRECT = 3

    def __init__(self):
        self.velocity = 0
        self.received_manual_direct_cmd_at = time.time()
        self.received_manual_inverse_cmd_at = time.time()
        self.command_expiration = .5   # seconds
        motor_count = 8
        self.manual_direct_command = [0]*motor_count
        self.manual_inverse_command = [0]*motor_count
        self.semi_autonomous_command = None
        self.mode = self.MANUAL_DIRECT
        self.target_mode = self.MANUAL_DIRECT
        self.mode_transitioning = False
        self.reset_arm_pos = False

    def mode_callback(self, msg):# Int8):
        """listens to HD_mode topic published by CS"""
        self.target_mode = msg.data
        self.mode_transitioning = True

    def taskCmdCallback(self, msg):# Task):
        """listens to task assignement topic published by detection"""

    def manual_cmd_callback(self, msg):# Int8MultiArray):
        """listens to HD_joints topic"""
        if self.mode == self.MANUAL_DIRECT:
            max_speed = 100
            self.manual_direct_command = [float(x)/max_speed for x in msg.data]
            self.received_manual_direct_cmd_at = time.time()
        elif self.mode == self.MANUAL_INVERSE:
            max_speed = 100
            self.manual_inverse_command = [float(x)/max_speed for x in msg.data]
            self.received_manual_inverse_cmd_at = time.time()

    def semi_autonomous_callback(self, msg):
        self.semi_autonomous_command = msg.data

    def send_task_cmd(self):
        """sends the last task command to the task executor and locks any other command until completion"""

    def send_manual_direct_cmd(self):
        """sends the last direct command to the motor control and locks any other command until completion"""
        if self.manual_direct_command_old(): 
            return
        msg = Float32MultiArray()
        msg.data = self.manual_direct_command
        if VERBOSE:
            rospy.logwarn("manager direct cmd :   " + str(msg.data))
        self.manual_direct_cmd_pub.publish(msg)

    def send_manual_inverse_cmd(self):
        """sends the last manual command to the manual control and locks any other command until completion"""
        if self.manual_inverse_command_old():
            return
        cmd = self.manual_inverse_command
        if cmd[0] != 0 or cmd[1] != 0 or cmd[2] != 0:
            msg = Float32MultiArray()
            msg.data = cmd[:3]
            msg.data.append(max(cmd[:3]))
            if VERBOSE:
                rospy.loginfo("manager pos man inv cmd :   " + str(msg.data))
            self.pos_manual_inverse_cmd_pub.publish(msg)
        elif cmd[3] != 0 or cmd[4] != 0 or cmd[5] != 0:
            msg = Float32MultiArray()
            msg.data = cmd[3:6]
            msg.data.append(max(cmd[3:6]))
            if VERBOSE:
                rospy.loginfo("manager orient man inv cmd :   " + str(msg.data))
            self.orient_manual_inverse_cmd_pub.publish(msg)

    def send_semi_autonomous_cmd(self):
        if self.semi_autonomous_command is not None:
            rospy.logwarn("SENDING SEMI AUTO CMD")
            task = Task()
            task.description = "btn"
            task.id = self.semi_autonomous_command
            self.semi_autonomous_command = None
            self.task_pub.publish(task)

    def updateWorld(self):
        """sends a world update to the trajectory planner"""
        # TODO

    def manual_direct_command_old(self):
        return time.time()-self.received_manual_direct_cmd_at > self.command_expiration
    
    def manual_inverse_command_old(self):
        return time.time()-self.received_manual_inverse_cmd_at > self.command_expiration

    def normal_loop_action(self):
        rospy.logwarn("MODE : " + str(self.mode))
        if self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.SEMI_AUTONOMOUS:
            self.send_semi_autonomous_cmd()
        elif self.mode == self.MANUAL_INVERSE:
            self.send_manual_inverse_cmd()
        elif self.mode == self.MANUAL_DIRECT:
            self.send_manual_direct_cmd()

    def transition_loop_action(self):
        if self.mode == self.AUTONOMOUS:
            pass
        elif self.mode == self.SEMI_AUTONOMOUS:
            pass
        elif self.mode == self.MANUAL_INVERSE:
            pass
        elif self.mode == self.MANUAL_INVERSE:
            pass

        transition_condition = True
        if transition_condition:
            self.mode_transitioning = False
            self.mode = self.target_mode

    def run(self):
        """main"""
        self.manual_direct_cmd_pub = rospy.Publisher('/arm_control/manual_direct_cmd', Float32MultiArray, queue_size=10)
        self.pos_manual_inverse_cmd_pub = rospy.Publisher('/arm_control/pos_manual_inverse_cmd', Float32MultiArray, queue_size=10)
        self.orient_manual_inverse_cmd_pub = rospy.Publisher('/arm_control/orient_manual_inverse_cmd', Float32MultiArray, queue_size=10)
        self.task_pub = rospy.Publisher('/arm_control/task_assignment', Task, queue_size=10)
        #self.reset_arm_pos_pub = rospy.Publisher('/arm_control/reset_arm_pos', Bool, queue_size=10)
        #self.set_zero_arm_pos_pub = rospy.Publisher('/arm_control/set_zero_arm_pos', Bool, queue_size=10)
        rospy.Subscriber("HD_Angles_sim", Int8MultiArray, self.manual_cmd_callback)
        rospy.Subscriber("CS_HD_mode_sim", Int8, self.mode_callback)
        rospy.Subscriber("fsm_state", Int16, self.semi_autonomous_callback)
        #rospy.Subscriber("HD_ManualVelocity", Float32, self.manualVelocityCallback)
        #rospy.Subscriber("HD_reset_arm_pos", Bool, self.resetCallback)
        #rospy.Subscriber("HD_set_zero_arm_pos", Bool, self.setZeroCallback)
        rate = rospy.Rate(25)   # 25hz
        if VERBOSE:
            rospy.logwarn("manager started")
        while not rospy.is_shutdown():
            if self.mode_transitioning:
                self.transition_loop_action()
            else:
                self.normal_loop_action()
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('HD_control_manager', anonymous=True)
        m = Manager()
        m.run()
        if VERBOSE:
            rospy.logwarn("manager finished")
    except rospy.ROSInterruptException:
        rospy.logerr("manager crashed")
        pass
