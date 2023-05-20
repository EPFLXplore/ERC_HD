import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray

class ARTagsPublisher(Node):
    def __init__(self):
        super().__init__('HD_ar_tags_publisher')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'HD/ar_tags', 10)


    def publish_detected_tags(self, tags):
        msg = Int8MultiArray()

        ## TODO ##
        # Convert tag info to an array


        self.publisher_.publish(msg)
