# Basic ROS 2 program to publish real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library

import sys

sys.path.append("./src/vision/vision")
from stereo_camera import StereoCamera
import os


class ImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_publisher")

        # Create the publisher. This publisher will publish an Image
        # to the HD/vision/video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(
            CompressedImage, "HD/vision/video_frames", 1
        )
        self.get_logger().info("Image Publisher Created")

        # We will publish a message every 0.1 seconds
        # timer_period = 0.04  # seconds

        # Create the timer
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        # self.cap = cv2.VideoCapture(0)
        # self.camera = StereoCamera()

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # def timer_callback(self):
    #     """
    #     Callback function.
    #     This function gets called every 0.1 seconds.
    #     """
    #     # Capture frame-by-frame
    #     # This method returns True/False as well
    #     # as the video frame.
    #     # ret, frame = self.cap.read()
    #     frame = self.camera.get_image()

    def publish(self, frame):
        """
        Publishes a frame to the video_frames topic
        """
        # if ret == True:
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        scale = 1
        frame = cv2.resize(frame, (0, 0), fx=scale, fy=scale)
        self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(frame))

        # Display the message on the console
        # self.get_logger().info("Publishing video frame")
