#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray



class PublisherSteer(Node):
    def __init__(self):
        super().__init__("publisher_pos_controller")
       
        controller_name = "hd_pos_controller"
        wait_sec_between_publish = 0.01
        self.goals = [[0.0, 0.0]]

        subscribe_topic = "/HD/cmds"
        self.subscription = self.create_subscription(
            Float64MultiArray, subscribe_topic, self.pos_callback, 1)

        publish_topic = "/" + controller_name + "/" + "commands"

        self.publisher_ = self.create_publisher(Float64MultiArray, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def pos_callback(self, msg):
        self.goals = [[msg.data[0],msg.data[1] ]]#,0.,0.,0.,0.,0.]]

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
