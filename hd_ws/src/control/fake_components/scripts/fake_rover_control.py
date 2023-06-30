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


class FakeRoverControl(Node):
    MOTOR_COUNT = 10     # number of motors
    MAX_VEL = [3.0, 1.5, 2.0, 4.0, 2.0, 3.0]    # max velocity of each motor in rad/s
    MAX_VEL = [x*10 for x in MAX_VEL]
    VELOCITY = 0
    POSITION = 1

    def __init__(self):
        super().__init__("fake_rover_control")
        self.state = JointState()
        self.init_state()
        self.state_pub = self.create_publisher(JointState, "/HD/kinematics/rover_joints_telemetry", 10)
        self.create_subscription(Float64MultiArray, "/HD/fsm/joint_vel_cmd", self.vel_cmd_callback, 10)     # TODO: change this topic

        self.control_mode = self.POSITION
        self.last_vel_cmd = time.time()

        self.cmd_velocities = [0.0]*self.MOTOR_COUNT
        self.timer = Timer()

    def init_state(self):
        self.state.position = [0.0]*self.MOTOR_COUNT
        self.state.velocity = [0.0]*self.MOTOR_COUNT
        self.state.effort = [0.0]*self.MOTOR_COUNT

    def vel_cmd_callback(self, msg: Float64MultiArray):
        self.last_vel_cmd = time.time()
        self.control_mode = self.VELOCITY
        self.timer.tick()

        data = list(msg.data) + [0.0]*max(0, self.MOTOR_COUNT-len(msg.data))
        self.cmd_velocities = [max_ * vel for max_, vel in zip(self.MAX_VEL, data)]

    def publish_state(self):
        msg = copy.deepcopy(self.state)
        self.state_pub.publish(msg)

    def vel_cmd_deprecated(self):
        return time.time()-self.last_vel_cmd > 1
    
    def update(self):
        if self.vel_cmd_deprecated():
            self.control_mode = self.POSITION

        if self.control_mode == self.VELOCITY:
            self.update_from_velocities()

    def update_from_velocities(self):
        self.state.velocity = self.cmd_velocities
        t = self.timer.tick()
        self.state.position = [pos + vel*t for pos, vel in zip(self.state.position, self.cmd_velocities)]

    def loop(self):
        rate = self.create_rate(25)
        while rclpy.ok():
            self.update()
            self.publish_state()
            rate.sleep()


def main():
    rclpy.init()
    node = FakeRoverControl()

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
