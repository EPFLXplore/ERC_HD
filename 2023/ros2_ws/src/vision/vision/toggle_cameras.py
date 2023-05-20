import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class HDToggleCamerasSubscriber:
    def __init__(self, node, publisher):
        self.parent_node = node
        self.publisher_ = publisher
        self.subscription_ = node.create_subscription(
            Int8,
            'ROVER/HD_toggle_cameras',
            self.callback,
            10
        )

    def callback(self, msg):
        if msg.data == 0: # Off
            self.publisher_.enabled_ = False
            print("Camera publisher disabled")
        elif msg.data == 1: # On
            self.publisher_.enabled_ = True
            print("Camera flux enabled")
