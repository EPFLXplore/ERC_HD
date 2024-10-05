from kinematics_utils.node_template import NodeTemplate
from .bota_sensor import BotaSerialSensor, BotaSerialSensorError
import sys
import geometry_msgs.msg as gmsg
import kinematics_utils.pose_tracker as pt
from kinematics_utils.quaternion_arithmetic_new import Quaternion, Point
import math
import time


class BotaPublsher(NodeTemplate):
    def __init__(self):
        super().__init__("bota_sensor")
        if len(sys.argv) > 1:
            try:
                self.sensor = BotaSerialSensor(sys.argv[1])
                self.sensor.start()
            except BotaSerialSensorError as expt:
                print('bota_serial_example failed: ' + expt.message)
                sys.exit(1)
        else:
            print('usage: bota_serial_example portname')
            sys.exit(1)
        
        deg2rad = lambda x: x/180*math.pi
        self.sensor_orientation = Quaternion.from_axis_angle([0, 0, 1], -deg2rad(45))  # relative to urdf end effector
        self.in_eef_frame = True
        self.torque_pub = self.create_publisher(gmsg.Point, "/HD/sensors/directional_torque", 10)
        pt.link_eef_pose_callback(self)
        time.sleep(1.0)
        self.offset_torque = self.sensor.get_directional_force()
    
    def get_torque(self) -> Point:
        torque = self.sensor.get_directional_force() - self.offset_torque
        torque = self.sensor_orientation.point_image(torque)
        if self.in_eef_frame:
            torque = pt.END_EFFECTOR_POSE.orientation.point_image(torque)
        return torque
    
    def loop_action(self):
        self.torque_pub.publish(self.get_torque().publishable())


def main():
    BotaPublsher.main()
