import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class ElementIdSubscriber:
    def __init__(self, node, publisher):
        self.parent_node = node
        self.subscription_ = node.create_subscription(
            Int8,
            'ROVER/element_id',
            self.callback,
            10
        )

    def callback(self, msg):
        # do something ?
        print(msg)