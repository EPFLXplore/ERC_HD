#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from custom_msg.msg import Motorcmds


class PublisherSteer(Node):
    def __init__(self):
        super().__init__("publisher_pos_controller")
       
        controller_name = "position_controller"
        wait_sec_between_publish = 0.4
        self.goals = [[0.0, 0.0, 0.0, 0.0]]

        # Subscribe to the topic that provides the velocities
        subscribe_topic = "/NAV/displacement"
        self.subscription = self.create_subscription(
            Motorcmds, subscribe_topic, self.steer_callback, 1)

        publish_topic = "/" + controller_name + "/" + "commands"

        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def steer_callback(self, msg):
        self.goals = [[msg.steer[0]*2*np.pi/65536,-msg.steer[1]*2*np.pi/65536, msg.steer[2]*2*np.pi/65536, -msg.steer[3]*2*np.pi/65536]]

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = self.goals[self.i]
        self.publisher_.publish(msg)
        self.i += 1
        self.i %= len(self.goals)


      


def main(args=None):
    rclpy.init(args=args)
    pub = PublisherSteer()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
