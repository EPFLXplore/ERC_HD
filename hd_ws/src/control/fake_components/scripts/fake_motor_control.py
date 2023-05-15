#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import threading
import copy
import time


class Timer:
    def __init__(self):
        self.last_tick = time.time()
    
    def tick(self):
        self.last_tick, t_prev = time.time(), self.last_tick
        return self.last_tick - t_prev
    
    def time_from_tick(self):
        return time.time() - self.last_tick


class FakeMotorControl(Node):
    MOTOR_COUNT = 6     # number of motors
    MAX_VEL = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]    # max velocity of each motor in rad/s

    def __init__(self):
        super().__init__("fake_motor_control")
        self.state = JointState()
        self.init_state()
        self.state_pub = self.create_publisher(JointState, "/HD/arm_control/joint_telemetry", 10)
        self.create_subscription(JointState, "/HD/kinematics/joint_cmd", self.moveit_cmd_callback, 10)
        self.create_subscription(Float64MultiArray, "/CS/vel_joint_cmd", self.vel_cmd_callback, 10)

        self.mimic = False   # wether to transmit commands from moveit directly to state (perfect motor)

        self.direct_manual_mode = True
        self.cmd_velocities = [0.0]*self.MOTOR_COUNT
        #self.cmd_velocities[0] = 1.0
        #self.cmd_velocities[2] = 1.0
        self.timer = Timer()

    def init_state(self):
        self.state.position = [0.0]*self.MOTOR_COUNT
        self.state.velocity = [0.0]*self.MOTOR_COUNT
        self.state.effort = [0.0]*self.MOTOR_COUNT

    def moveit_cmd_callback(self, msg: JointState):
        if self.mimic:
            self.state.position[:len(msg.position)] = msg.position
            self.state.velocity[:len(msg.velocity)] = msg.velocity
            self.state.effort[:len(msg.effort)] = msg.effort

    def vel_cmd_callback(self, msg: Float64MultiArray):
        data = list(msg.data) + [0.0]*max(0, self.MOTOR_COUNT-len(msg.data))
        self.cmd_velocities = [max_ * vel for max_, vel in zip(self.MAX_VEL, data)]

    def publish_state(self):
        msg = copy.deepcopy(self.state)
        self.state_pub.publish(msg)

    def update_from_velocities(self):
        self.state.velocity = self.cmd_velocities
        t = self.timer.tick()
        self.state.position = [pos + vel*t for pos, vel in zip(self.state.position, self.cmd_velocities)]

    def loop(self):
        rate = self.create_rate(25)
        while rclpy.ok():
            if self.direct_manual_mode:
                self.update_from_velocities()
            self.publish_state()
            rate.sleep()


def main():
    rclpy.init()
    node = FakeMotorControl()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
