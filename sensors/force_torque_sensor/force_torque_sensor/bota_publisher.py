from kinematics_utils.node_template import NodeTemplate
from .bota_sensor import BotaSerialSensor, BotaSerialSensorError
import sys
import geometry_msgs.msg as gmsg
import kinematics_utils.pose_tracker as pt
from kinematics_utils.quaternion_arithmetic_new import Quaternion


class BotaPublsher(NodeTemplate):
    def __init__(self):
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
        
        self.sensor_orientation = Quaternion()  # relative to urdf end effector
        self.in_eef_frame = True
        self.torque_pub = self.create_publisher(gmsg.Point, "/HD/sensors/directional_torque", 10)
        pt.link_eef_pose_callback(self)
        
    def loop_action(self):
        torque = self.sensor.get_directional_force()
        torque = self.sensor_orientation.point_image(torque)
        if self.in_eef_frame:
            torque = pt.END_EFFECTOR_POSE.orientation.point_image(torque)
        self.torque_pub.publish(torque.publishable())



def main():
    BotaPublsher.main()
