# class publisher contains all publishers for the vision node
import sys

sys.path.append("./src/vision/vision")

import rclpy
from rclpy.node import Node
from image_publisher import ImagePublisher
from target_pose_publisher import TargetPosePublisher
from target_publisher import TargetPublisher


class Publisher(Node):
    def __init__(self):
        super().__init__("publisher")

        self.image_publisher = ImagePublisher()
        self.pose_target_publisher = TargetPosePublisher()
        self.target_publisher = TargetPublisher()

        self.get_logger().info("Main Publisher Created")

        timer_period = 0.1

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target = 0

    def timer_callback(self):
        pass