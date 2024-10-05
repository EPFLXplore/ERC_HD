#!/usr/bin/env python3

from kinematics_utils.node_template import NodeTemplate
from input_handling.gamepad import GamePadConfig
from kinematics_utils.quaternion_arithmetic_new import Point
import geometry_msgs.msg as gmsg
import kinematics_utils.pose_tracker as pt
from kinematics_utils.pose_corrector_new import POSE_CORRECTOR as pc


class FakeTorqueSensor(NodeTemplate):
    def __init__(self):
        super().__init__("fake_bota_sensor")
        self.torque = Point()
        self.input_config = GamePadConfig()
        self.input_config.bind(GamePadConfig.RH, self.torque.set_x, "x")
        self.input_config.bind(GamePadConfig.RV, lambda y: self.torque.set_y(-y), "y")
        self.input_config.bind(GamePadConfig.R2, self.torque.set_z, "z")
        self.input_config.bind(GamePadConfig.L2, lambda z: self.torque.set_z(-z), "z")
        self.input_config.background_loop()
        
        self.in_eef_frame = True

        self.torque_pub = self.create_publisher(gmsg.Point, "/HD/sensors/directional_torque", 10)
        pt.link_eef_pose_callback(self)
        
    def loop_action(self):
        torque = self.torque
        if self.in_eef_frame:
            torque = pc.gripper_to_abs_vector(torque)
        torque *= 100
        print(torque)
        self.torque_pub.publish(torque.publishable())


if __name__ == "__main__":
    FakeTorqueSensor.main(hz=30)
