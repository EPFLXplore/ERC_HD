# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

from .interfaces.monocular_camera_interface import MonocularCameraInterface
from .camera_factory import CameraFactory
import time

import pyrealsense2 as rs

class BareCameraNode(Node):
    """
    Create an CameraNode class, publishes video frames to the video_frames topic
    """

    def __init__(self):
        super().__init__("camera_node")


        self.camera =  CameraFactory.create_camera('realsense_stereo')


        # to the HD/vision/video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(CompressedImage, "/HD/bare_camera/rgb", 1)
        self.get_logger().info("Image Publisher Created")

        # We will publish a message every 0.1 seconds
        timer_period = 1/6  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.rgb_callback)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # def timer_callback(self):

    def rgb_callback(self):
        """
        Callback function.
        Publishes a frame to the video_frames topic
        """
        color = self.camera.get_image()

        msg = self.bridge.cv2_to_compressed_imgmsg(color)

        self.publisher_.publish(msg)
        # self._logger.info('Publishing RGBD image')


def main(args=None):
    rclpy.init(args=args)

    camera_node = BareCameraNode()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


"""
README

Test:
- convert rbdf message to 2 compressed images and try tiff
- 

"""
