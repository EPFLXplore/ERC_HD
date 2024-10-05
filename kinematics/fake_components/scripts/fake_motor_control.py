#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState
from custom_msg.msg import MotorCommand
import threading
import copy
import time
import array
from math import pi


def pad(l: list, length: int, default:float = 0.0) -> list:
    if len(l) >= length:
        return l
    return l + [default] * (length-len(l))


def list_add(l1: list, l2: list) -> list:
    return [a+b for a, b in zip(l1, l2)]

def list_neg(l: list) -> list:
    return [-a for a in l]

def list_mul(l: list, x: float) -> list:
    return [a*x for a in l]


class Timer:
    def __init__(self):
        self.last_tick = time.time()
    
    def tick(self):
        self.last_tick, t_prev = time.time(), self.last_tick
        return self.last_tick - t_prev
    
    def time_from_tick(self):
        return time.time() - self.last_tick


class FakeMotorControl(Node):
    MOTOR_COUNT = 10     # number of motors
    rads2rpm = lambda x: x/pi*30
    rpm2rads = lambda x: x*pi/30
    # MAX_VEL = list(map(rpm2rads, pad([1.83, 1.377, 1.377, 1.83, 1.83, 1.83, 0.15], MOTOR_COUNT)))    # max velocity of each motor in rad/s
    MAX_VEL = pad([0.2, 0.13, 0.13, 0.2, 0.2, 0.2], MOTOR_COUNT)    # max velocity of each motor in rad/s
    MAX_VEL = list_mul(MAX_VEL, 1)
    # POSITION_OFFSETS = pad([0, -0.959505, -2.424073, 0, -1.27857, -1.88833], MOTOR_COUNT)
    POSITION_OFFSETS = [0.0] * MOTOR_COUNT
    VELOCITY = 0
    POSITION = 1

    def __init__(self):
        super().__init__("fake_motor_control")
        self.state = JointState()
        self.init_state()
        self.state_pub = self.create_publisher(JointState, "/HD/motor_control/joint_telemetry", 10)
        self.create_subscription(Float64MultiArray, "/HD/kinematics/joint_pos_cmd", self.moveit_cmd_callback, 10)
        self.create_subscription(Float64MultiArray, "/HD/fsm/joint_vel_cmd", self.vel_cmd_callback, 10)
        self.motor_command_pub = self.create_publisher(MotorCommand, "HD/kinematics/single_joint_cmd", 10)
        self.create_subscription(MotorCommand, "HD/kinematics/single_joint_cmd", self.invidual_motor_command_callback, 10)

        self.control_mode = self.VELOCITY
        self.last_vel_cmd = time.time()

        self.cmd_velocities = [0.0]*self.MOTOR_COUNT
        self.target_positions = [0.0]*self.MOTOR_COUNT
        self.scan_for_positions()
        self.timer = Timer()

    def init_state(self):
        self.state.position = [0.0]*self.MOTOR_COUNT
        # self.state.position = pad([0.0, -0.9199, -0.7463, -0.0174, -1.2323, 1.2323], self.MOTOR_COUNT)
        # self.state.position = pad([0.0, -0.9199, -0.7463 + 0.3, -0.0174, -1.2323 + 0.3, 1.2323], self.MOTOR_COUNT)
        # self.state.position = list_add(self.state.position, list_neg(self.POSITION_OFFSETS))
        self.state.position = pad([0.0, 0.33161255787892263, 2.1816615649929116, 0.0, 0.2617993877991494, 0.0], self.MOTOR_COUNT)
        self.state.velocity = [0.0]*self.MOTOR_COUNT
        self.state.effort = [0.0]*self.MOTOR_COUNT

    def scan_for_positions(self):
        make_list = lambda arr: [*arr]
        self.target_positions = make_list(self.state.position)
        
    def moveit_cmd_callback(self, msg: Float64MultiArray):
        if self.control_mode != self.POSITION:
            return
        # self.state.position[:len(msg.data)] = array.array('d', [msg_pos - offset for msg_pos, offset in zip(msg.data, self.POSITION_OFFSETS)])
        self.target_positions[:len(msg.data)] = [msg_pos - offset for msg_pos, offset in zip(msg.data, self.POSITION_OFFSETS)]
        # self.get_logger().info("new target positions: " + str(self.target_positions))

    def vel_cmd_callback(self, msg: Float64MultiArray):
        if self.vel_cmd_deprecated:
            self.timer.tick()
            
        self.last_vel_cmd = time.time()
        self.control_mode = self.VELOCITY

        data = list(msg.data) + [0.0]*max(0, self.MOTOR_COUNT-len(msg.data))
        self.cmd_velocities = [max_ * vel for max_, vel in zip(self.MAX_VEL, data)]

    def invidual_motor_command_callback(self, msg: MotorCommand):
        cmd_mode = {
            MotorCommand.POSITION: "position",
            MotorCommand.VELOCITY: "velocity",
            MotorCommand.TORQUE: "torque",
        }[msg.mode]
        self.get_logger().info(f"Fake motor control received {cmd_mode} command for {msg.name} of value {msg.command}")
        
    def publish_state(self):
        msg = copy.deepcopy(self.state)
        
        # correct for offset
        for i in range(min(len(msg.position), len(self.POSITION_OFFSETS))):
            msg.position[i] += self.POSITION_OFFSETS[i]
            
        self.state_pub.publish(msg)

    def vel_cmd_deprecated(self):
        return time.time()-self.last_vel_cmd > 1
    
    def update(self):
        if self.control_mode == self.POSITION:
            self.reach_target_positions()

        elif self.control_mode == self.VELOCITY:
            if self.vel_cmd_deprecated():
                self.scan_for_positions()
                self.control_mode = self.POSITION
                return
            self.update_from_velocities()

    def reach_target_positions(self):
        # mimicking a cyclic synchronous position mode: tries to reach the target position at the max allowed speed
        t = self.timer.tick()
        for i in range(self.MOTOR_COUNT):
            diff = self.target_positions[i] - self.state.position[i]
            dir = -1 if diff < 0 else 1
            self.state.position[i] += dir * min(abs(diff), t*self.MAX_VEL[i])

    def update_from_velocities(self):
        self.state.velocity = self.cmd_velocities
        t = self.timer.tick()
        self.state.position = [pos + vel*t for pos, vel in zip(self.state.position, self.cmd_velocities)]

    def loop(self):
        rate = self.create_rate(50)
        while rclpy.ok():
            self.update()
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
