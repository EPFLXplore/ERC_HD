#!/usr/bin/env python3

from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from custom_msg.msg import Task
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8
from sensor_msgs.msg import JointState

import keyboard
import threading
from typing import Any, Union, List
from typing_extensions import Self
import math
import kwak
from kwak.fullscreen import *
from kwak.interface import *
from kwak.text import BText
from kwak.text_input import TextInput
from kwak.text_box import TextBox
from kwak.select import ComboBox
import pygame


#pygame.init()
BIGARM = pygame.image.load(kwak_path("images/bigarm.png"))
pygame.display.set_icon(BIGARM)
pygame.display.set_caption("HDDDD")
window = pygame.display.set_mode((500, 250), pygame.RESIZABLE)
kwak.init(window, init_pygame=True)
bg_color = (0, 35, 35), (0, 35, 35), (0, 115, 60)
text_color = (255, 255, 255), (255, 255, 255), (255, 255, 255)
outline_color = (200, 200, 255), (100, 255, 100), (100, 255, 100)
p = Palette(text_color, bg_color, outline_color)
Interface.get_instance().set_global_palette(p)
Interface.get_instance().set_fps_visible(True)


@dataclass
class JointStateLabels:
    position: BText
    velocity: BText
    effort: BText
    
    def update(self, state: JointState, ind: int):
        self.position.set_text(f"{state.position[ind]:.3f}")
        self.velocity.set_text(f"{state.velocity[ind]:.3f}")
        self.effort.set_text(f"{state.effort[ind]:.3f}")


class ControlStation(Node):
    HD_MODES = {
        "Idle": -1,
        "Direct": 1,
        "Manual inverse": 0,
        "Semi auto": 2
    }
    
    def __init__(self):
        super().__init__("fake_cs_kwak")
        self._create_ros_interfaces()
        self.motor_count = 6
        self.motor_telem: JointState = JointState(position=[0.0]*self.motor_count, velocity=[0.0]*self.motor_count, effort=[0.0]*self.motor_count)
        self.cmd_velocities = Float32MultiArray(data=[0.0]*(self.motor_count+1))
        self.state_labels: List[JointStateLabels] = []
        self._create_app()
    
    def _create_app(self):
        font = pygame.font.SysFont("arial", 15)
        telem_layout = GridLayout()
        self.motor_telem_labels = []
        telem_layout.add_item(BText(text="Joints"), 0, 0)
        telem_layout.add_item(BText(text="Position"), 1, 0)
        telem_layout.add_item(BText(text="Velocity"), 2, 0)
        telem_layout.add_item(BText(text="Effort"), 3, 0)
        for i in range(self.motor_count):
            joint_label = BText(text=f"Joint{i+1}")
            state_labels = JointStateLabels(BText(), BText(), BText())
            state_labels.update(self.motor_telem, i)
            self.state_labels.append(state_labels)
            telem_layout.add_item(joint_label, 0, i+1)
            telem_layout.add_item(state_labels.position, 1, i+1)
            telem_layout.add_item(state_labels.velocity, 2, i+1)
            telem_layout.add_item(state_labels.effort, 3, i+1)
        
        cmd_layout = VLayout()
        hlay = HLayout()
        mode_label = BText(text="HD mode:")
        selection = ComboBox(font=font)
        selection.set_options(RectButton.list_of_options(strings=list(self.HD_MODES)))
        selection.connect("option_select", self.mode_update, nb_intern_params=1)
        hlay.add_rows([mode_label, selection])
        cmd_layout.add_row(hlay)
        #vlay = VLayout()
        gd_lay = GridLayout()
        for i in range(self.motor_count):
            joint_label = BText(text=f"Joint{i+1} position:")
            box = TextBox(text="0.0")
            gd_lay.add_item(joint_label, 0, i)
            gd_lay.add_item(box, 1, i)
            # joint_pos_cmd = TextInput(label_text=f"Joint{i} position:")
            # vlay.add_row(joint_pos_cmd)
            box.connect("pushed", self.velocity_update, params=(i,), nb_intern_params=1)
        cmd_layout.add_row(gd_lay)
        
        tabs = [("Telemetry", telem_layout), ("Commands", cmd_layout)]
        self.app = App(tabs=tabs, font=font)
        Interface.get_instance().add_widget(self.app)
    
    def _create_ros_interfaces(self):
        self.joint_vel_cmd_pub = self.create_publisher(Float32MultiArray, "/CS/HD_gamepad", 10)
        self.man_inv_axis_pub = self.create_publisher(Float32MultiArray, "/ROVER/HD_man_inv_axis", 10)
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        
        self.create_subscription(JointState, "HD/motor_control/joint_telemetry", self.telem_callback, 10)
    
    def velocity_update(self, motor_ind: int, str_vel: str):
        vel = float(str_vel)
        self.cmd_velocities.data[motor_ind+1] = vel
    
    def mode_update(self, str_mode: str):
        mode = self.HD_MODES[str_mode]
        self.mode_change_pub.publish(Int8(data=mode))
        
    def telem_callback(self, msg: JointState):
        self.motor_telem = msg
        for i in range(self.motor_count):
            self.state_labels[i].update(self.motor_telem, i)
    
    def publish_cmd(self):
        self.joint_vel_cmd_pub.publish(self.cmd_velocities)
        
    def loop(self):
        #threading.Thread(target=Interface.get_instance().run, daemon=True).start()
        rate = self.create_rate(10)
        while rclpy.ok():
            # self.read_inputs()
            self.publish_cmd()
            interface = Interface.get_instance()
            interface.update()
            window.fill((0, 0, 0))
            interface.display()
            pygame.display.update()
            #rate.sleep()


def main():
    rclpy.init()
    node = ControlStation()

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
