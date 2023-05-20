import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray

class ARTagsPublisher:
    def __init__(self, node):
        self.parent_node = node
        self.publisher_ = node.create_publisher(Int8MultiArray, 'HD/ar_tags', 10)

    def get_logger(self):
        return self.parent_node.get_logger()
    
    def publish_detected_tags(self, tag_ids):
        msg = Int8MultiArray()
        msg.data = [0, 0, 0, 0]

        for id in tag_ids:
            msg.data[id] = 1

        self.publisher_.publish(msg)
