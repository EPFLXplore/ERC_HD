import cv2

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
from custom_msg.msg import CompressedRGBD  # Custom message type

import numpy as np


from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import Pose

from .handlers.pose_msg import PoseMsg
from .pipelines.pipeline_factory import PipelineFactory
from .pipelines.pipeline_interface import PipelineInterface
from custom_msg.srv import CameraParams
from custom_msg.srv import ButtonPressControlPanel
import re
import yaml
import os
from .get_interfaces import GetInterfaces

class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.config_path = f"{os.getcwd()}/src/sensors/camera/configs/realsense_config.yaml"
        self.load_config()


        self.button_pattern = r"^\d[ud]$"  # used to match button service requests to distinguish them from switching pipeline request

        # Initialize Camera Info
        self.camera_matrix = None
        self.dist_coeffs = None
        self._get_camera_params()

        # Initialize Camera Info
        self.camera_matrix = None
        self.dist_coeffs = None
        self._get_camera_params()

        # Initialize Pipeline with calibration data if available
        self.pipeline: PipelineInterface = PipelineFactory.create_pipeline(
            "buttonsA",
            self,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
        )

        # Subscribers
        # self.rgb_sub = self.create_subscription(
        #     CompressedImage, GetInterfaces.get('hd_camera_rgb'), self.rgb_callback, 10
        # )
        # self.depth_sub = self.create_subscription(
        #     CompressedImage, "HD/camera/depth", self.depth_callback, 10
        # )
        self.rgbd_sub = self.create_subscription(
            CompressedRGBD, GetInterfaces.get('hd_camera_rgbd'), self.rgbd_callback, 1
        )

        # Publishers
        self.processed_rgb_pub = self.create_publisher(
            CompressedImage, GetInterfaces.get('hd_perception_rgb'), 1
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.srv = self.create_service(
            ButtonPressControlPanel,
            GetInterfaces.get('hd_gui_buttons'),
            self.handle_button_press,
        )

        self.get_logger().info("Perception Node started with buttons gang pipeline")

    def _get_camera_params(self):
        """Request the camera matrix and distortion coefficients from the camera node."""
        client = self.create_client(CameraParams, "/HD/camera/params")

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for camera_params service...")

        req = CameraParams.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self._process_camera_params(future.result())
        else:
            self.get_logger().error("Failed to get camera parameters")

    def _process_camera_params(self, camera_params):
        """Process camera parameters to extract camera matrix and distortion coefficients."""
        # Create camera matrix from intrinsics
        self.camera_matrix = np.array(
            [
                [camera_params.fx, 0, camera_params.cx],
                [0, camera_params.fy, camera_params.cy],
                [0, 0, 1],
            ]
        )
        # Convert distortion coefficients to numpy array
        self.dist_coeffs = np.array(camera_params.distortion_coefficients)

    def rgb_callback(self, rgb_msg):
        """Handle incoming RGB images."""
        rgb = self.bridge.compressed_imgmsg_to_cv2(rgb_msg)
        # Process the RGB image if needed

    def depth_callback(self, depth_msg):
        """Handle incoming depth images."""
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "mono16")
        # Process the depth image if needed

    def rgbd_callback(self, rgbd_msg: CompressedRGBD):
        rgb = self.bridge.compressed_imgmsg_to_cv2(rgbd_msg.color)
        depth_image = self.bridge.imgmsg_to_cv2(rgbd_msg.depth, "mono16")

        # Run the pipeline with RGB and depth images
        self.pipeline.run_rgbd(rgb, depth_image)

        # Convert the processed image back to ROS message and publish
        rgb_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)
        self.processed_rgb_pub.publish(rgb_msg)

    def handle_button_press(self, request, response):
        selected_button = request.button_name
        self.get_logger().info(f"button / pipeline : {request.button_name} requested")
        is_button_request = bool(re.fullmatch(self.button_pattern, selected_button))
        if is_button_request:
            if not self.pipeline.name() == "buttonsA":
                self.pipeline = PipelineFactory.create_pipeline(
                    "buttonsA",
                    self,
                    camera_matrix=self.camera_matrix,
                    dist_coeffs=self.dist_coeffs,
                )
            self.pipeline.set_button(selected_button)
        else:
            self.pipeline = PipelineFactory.create_pipeline(
                    selected_button,
                    self,
                    camera_matrix=self.camera_matrix,
                    dist_coeffs=self.dist_coeffs,
                )

        response.response = f"Button {request.button_name} was pressed"

        return response
    
    def load_config(self):
        with open(self.config_path, "r") as file:
            self.config = yaml.safe_load(file)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()