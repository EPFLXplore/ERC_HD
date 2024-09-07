# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from custom_msg.msg import CompressedRGBD
from custom_msg.srv import CameraParams


import yaml
from .interfaces.monocular_camera_interface import MonocularCameraInterface
from .camera_factory import CameraFactory
import time
import os
import pyrealsense2 as rs

class CameraNode(Node):
    """
    Create an CameraNode class, publishes video frames to the video_frames topic
    """

    def __init__(self):
        super().__init__("camera_node")
        self.config_path = f"{os.getcwd()}/src/sensors/camera/configs/realsense_config.yaml"
        self.load_config()

        camera_type = self.declare_parameter("camera_type", 'realsense_stereo').value

        self.camera_intrinsics_srv = self.create_service(
            CameraParams,
            "/HD/camera/params",
            self.camera_params_callback,
        )

        self.camera =  CameraFactory.create_camera(camera_type)
        timer_period = 1.0 / self.camera.fps


        # to the HD/vision/video_frames topic. The queue size is 10 messages.
        if self.config['publish_straigth_to_cs']:
            self.publisher_ = self.create_publisher(CompressedImage, "/HD/camera/rgb", 1)
            self.timer = self.create_timer(timer_period, self.rgb_callback)
        else:
            self.publisher_ = self.create_publisher(CompressedRGBD, "/HD/camera/rgbd", 1)
            self.timer = self.create_timer(timer_period, self.rgbd_callback)

        self.get_logger().info("Image Publisher Created")


        # Create the timer

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # def timer_callback(self):

    def rgbd_callback(self):
        """
        Callback function.
        Publishes a frame to the video_frames topic
        """
        color, depth = self.camera.get_rgbd()

        msg = CompressedRGBD()

        depth_msg = self.bridge.cv2_to_imgmsg(depth, "mono16")

        msg.depth = depth_msg

        msg.color = self.bridge.cv2_to_compressed_imgmsg(color)

        self.publisher_.publish(msg)
        # self._logger.info('Publishing RGBD image')


    def rgb_callback(self):
        """
        Callback function.
        Publishes a frame to the video_frames topic
        """
        color, depth = self.camera.get_rgbd()
        msg = self.bridge.cv2_to_compressed_imgmsg(color)

        self.publisher_.publish(msg)
        # self._logger.info('Publishing RGBD image')

    def camera_params_callback(self, request, response):
        intrinsics = self.camera.get_intrinsics()
        depth_scale = self.camera.get_depth_scale()
        distortion_coefficients = self.camera.get_coeffs()
        response.depth_scale = depth_scale
        response.fx = intrinsics["fx"]
        response.fy = intrinsics["fy"]
        response.cx = intrinsics["cx"]
        response.cy = intrinsics["cy"]
        response.distortion_coefficients = distortion_coefficients
        self.get_logger().info(
            f"Provided intrinsics: fx={response.fx}, fy={response.fy}, cx={response.cx}, cy={response.cy}"
        )
        self.get_logger().info(
            f"Provided distortion coefficients: {response.distortion_coefficients}"
        )
        self.get_logger().info(
            f"Provided depth scale: {response.depth_scale}"
        )
        return response
    
    def load_config(self):
        with open(self.config_path, "r") as file:
            self.config = yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraNode()
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
