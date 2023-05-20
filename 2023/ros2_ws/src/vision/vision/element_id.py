import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class ElementIdSubscriber(Node):
    def __init__(self, publisher):
        super().__init__('HD_element_id_subscriber')
        self.subscription_ = self.create_subscription(
            Int8,
            'ROVER/element_id',
            self.callback,
            10
        )

    def callback(self, msg):
        # do something ?
        print(msg)