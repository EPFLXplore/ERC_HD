import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pyrealsense2 import pyrealsense2 as rs
import numpy as np

from cv_bridge import CvBridge


class CameraFluxPublisher(Node):
    def __init__(self):
        super().__init__('HD_camera_flux_publisher')
        self.publisher_ = self.create_publisher(Image, 'HD/camera_flux', 10)
        self.bridge_ = CvBridge()
        self.enabled_ = False  # Flag to indicate if the publisher is enabled

        # Configure Realsense camera
        self.pipeline_ = rs.pipeline()
        self.config_ = rs.config()

        # enable_stream(rs2_stream stream_type, int width, int height, rs2_format format=RS2_FORMAT_ANY, int framerate=0)
        self.config_.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline_.start(self.config_)

    def publish_frame(self):

        if not self.enabled_:
            return
        
        frames = self.pipeline_.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            # Convert the Realsense frame to an OpenCV image
            img = np.asanyarray(color_frame.get_data())

            # Convert the OpenCV image to a ROS Image message
            img_msg = self.bridge_.cv2_to_imgmsg(img, encoding='bgr8')

            # Set the header information of the Image message
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'

            # Publish the Image message
            self.publisher_.publish(img_msg)