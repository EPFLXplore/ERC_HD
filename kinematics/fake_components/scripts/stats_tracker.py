#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
from custom_msg.msg import TargetInstruction
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.quaternion_arithmetic_new as qan
# import kinematics_utils.pose_corrector as pc
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc
import kinematics_utils.pose_tracker as pt
import math
from math import pi
import time
import os
from datetime import datetime


class Stats(Node):
    def __init__(self):
        super().__init__("stats_tracker")
        self.distance_traveled = 0.0
        self.dof_count = 6
        self.angular_rotation_performed = [0.0]*self.dof_count
        self.current_joint_positions = [0.0]*self.dof_count
        self.must_scan_positions = True
        self.create_subscription(Pose, "/HD/kinematics/eef_pose", pt.eef_pose_callback, 10)
        self.create_subscription(JointState, "/HD/motor_control/joint_telemetry", self.joint_telemetry_callback, 10)
        self.start_datetime = datetime.now()
        self.start_time = time.time()
    
    def joint_telemetry_callback(self, msg: JointState):
        new_joint_positions = msg.position
        if self.must_scan_positions:
            self.must_scan_positions = False
        else:
            for i in range(self.dof_count):
                self.angular_rotation_performed[i] += abs(new_joint_positions[i]-self.current_joint_positions[i])
        self.current_joint_positions = new_joint_positions
        
    def loop(self):
        rate = self.create_rate(100)
        last_eef_pose = pc.correct_eef_pose()
        while rclpy.ok():
            new_eef_pose = pc.correct_eef_pose()
            delta_pos = new_eef_pose.position - last_eef_pose.position
            self.distance_traveled += delta_pos.norm()
            last_eef_pose = new_eef_pose
            rate.sleep()
    
    def save(self):
        save_dir = "running_logs"
        os.makedirs(save_dir, exist_ok=True)
        file_name = str(self.start_datetime)
        total_time = time.time() - self.start_time
        file_content = "\n".join([
            f"Start time: {str(self.start_datetime)}"
            f"Running time [s]: {total_time}",
            f"End effector distance traveled [m]: {self.distance_traveled}",
            f"Angular rotation per joint performed [rad]:",
            *(f"\tJ{i+1}: {self.angular_rotation_performed[i]}" for i in range(self.dof_count))
        ])
        with open(os.path.join(save_dir, file_name), 'w') as f:
            f.write(file_content)


def main(args=None):
    rclpy.init(args=args)
    stats = Stats()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(stats, ), daemon=True)
    thread.start()

    try:
        stats.loop()
    except KeyboardInterrupt:
        pass
    
    stats.save()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()