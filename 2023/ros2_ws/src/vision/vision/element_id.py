import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class ElementIdSubscriber:
    def __init__(self, node):
        self.parent_node = node
        self.element_id = -1
        self.subscription_ = node.create_subscription(
            Int8,
            'ROVER/element_id',
            self.callback,
            10
        )

    def callback(self, msg):
        self.element_id = msg
        print(msg)