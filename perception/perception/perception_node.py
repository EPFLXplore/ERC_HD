import threading
import queue
import time
from time import sleep
import cv2
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CompressedImage  # Image is the message type
from custom_msg.msg import CompressedRGBD  # Custom message type
from custom_msg.msg import Model  

from rclpy.callback_groups import ReentrantCallbackGroup
from custom_msg.srv import InitializeModel

import numpy as np
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from custom_msg.srv import CameraParams, ButtonPressControlPanel
import re

from .pipelines.pipeline_interface import PipelineInterface
from .pipelines.pipeline_factory import PipelineFactory
import os
import yaml


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        # used to match button service requests to distinguish them from switching pipeline request
        self.pipeline = None
        self.config_path = f"{os.getcwd()}/src/sensors/camera/configs/realsense_config.yaml"
        self.load_config()
        self.button_pattern = r"^\d[ud]$"  

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize Queue for Producer-Consumer model
        self.queue = queue.Queue(maxsize=10)

        # ROS 2 Subscriptions

        # self.rgbd_sub = self.create_subscription(
        #     CompressedRGBD, "/HD/camera/rgbd", self.rgbd_callback, 10
        # )  # TODO ?? no more need 
        if self.config['bypass_model_node']:
            self.processed_rgb_sub = self.create_subscription(
                CompressedRGBD, "/HD/camera/rgbd", self.rgbd_callback, 1)
        else:
            self.processed_rgb_sub = self.create_subscription(
                Model, "/HD/model/image", self.rgbd_callback, 1
            )

        # ROS 2 Publishers
        self.processed_rgb_pub = self.create_publisher(
            CompressedImage, "/HD/perception/image", 1
        )

        # ROS 2 Services
        self.srv = self.create_service(
            ButtonPressControlPanel,
            "/gui_node/button_press_service",
            self.handle_button_press,
        )

        # ROS 2 Clients 
        reentrant_callback_group = ReentrantCallbackGroup()

        # Client (to initialize model path)
        self.change_model_client = self.create_client(
            InitializeModel, "/HD/model_server/init_model", callback_group=reentrant_callback_group    # TODO ?? how/when to call this ? 
        )

        self.camera_params_client = self.create_client(
            CameraParams, "/HD/camera/params", callback_group=reentrant_callback_group    
        )

        # Initialize Camera Info
        self.camera_matrix = None
        self.depth_scale = None
        self._get_camera_params()

        # Initialize Pipeline 
        self.pipeline: PipelineInterface = PipelineFactory.create_pipeline(
            "buttonsA", self, camera_matrix=self.camera_matrix, camera_depth_scale=self.depth_scale
        )

        self.get_logger().info(f"Perception Node started with {self.pipeline.name()} pipeline")


    def _get_camera_params(self):
        """Request the camera matrix and distortion coefficients from the camera node."""
        self.get_logger().info("Waiting for camera_params service...")

        while not self.camera_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for camera_params service...")

        req = CameraParams.Request()
        future = self.camera_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self._process_camera_params(future.result())
        else:
            self.get_logger().error("Failed to get camera parameters")


    def _process_camera_params(self, camera_params):
        """Process camera parameters to extract camera matrix and distortion coefficients."""
        self.camera_matrix = np.array(
            [
                [camera_params.fx, 0, camera_params.cx],
                [0, camera_params.fy, camera_params.cy],
                [0, 0, 1],
            ]
        )
        self.dist_coeffs = np.array(camera_params.distortion_coefficients)
        self.depth_scale = camera_params.depth_scale
        # self.get_logger().info(f"Fx: {camera_params.fx}, fy: {camera_params.fy}")
        # self.get_logger().info(f"Depth scale: {camera_params.depth_scale}")

        self.get_logger().info("Camera parameters successfully saved")


    def rgbd_callback(self, model_msg: Model):
        """Producer: Receives images and adds them to the queue."""
        # Convert compressed image to OpenCV format
        self.get_logger().info('rgbd_callback')
        # rgb = self.bridge.compressed_imgmsg_to_cv2(model_msg.image.color)
        # depth_image = self.bridge.imgmsg_to_cv2(model_msg.image.depth, "mono16")
        rgb = self.bridge.compressed_imgmsg_to_cv2(model_msg.color)
        depth_image = self.bridge.imgmsg_to_cv2(model_msg.depth, "mono16")
        
        

        if self.pipeline is None:
            self.get_logger().info('The pipeline is not yet initialized')
            rgb_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)
            self.processed_rgb_pub.publish(rgb_msg)

        else:

            self.get_logger().info('Run through pipeline: START')
            self.pipeline.run_rgbd(rgb, depth_image, model_msg.segmentation_data)
            self.get_logger().info('Run through pipeline: END')

            # Convert the processed image back to ROS message and publish
            rgb_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)
            self.processed_rgb_pub.publish(rgb_msg)
            self.get_logger().info('Published annotated image')


    def handle_button_press(self, request, response):
        """Handle button press service request."""
        selected_button = request.button_name
        self.get_logger().info(f"button / pipeline : {request.button_name} requested")
        is_button_request = bool(re.fullmatch(self.button_pattern, selected_button))
        if is_button_request:
            if not self.pipeline.name() == "buttonsA":
                self.pipeline = PipelineFactory.create_pipeline(
                    "buttonsA", self, camera_matrix=self.camera_matrix, dist_coeffs=self.dist_coeffs
                )
            self.pipeline.set_button(selected_button)
        else:
            self.pipeline = PipelineFactory.create_pipeline(
                selected_button, self, camera_matrix=self.camera_matrix, dist_coeffs=self.dist_coeffs
            )

            # Service request for new model path
            self.change_model(name=request.button_name)

        response.response = f"Button {request.button_name} was pressed"
        return response
    

    def change_model(self, name):
        request = InitializeModel.Request()
        request.model_path = "src/perception/" + name + ".pt"

        self.get_logger().info(f"Service request to change model to {name}")
        start_time = time.time()

        self.response = None

        future = self.change_model_client.call_async(request)
        future.add_done_callback(lambda future: self._handle_response(future, start_time))

        try:
            self.response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        if self.response.success:
            self.get_logger().info("Model changed!")
        else:
            self.get_logger().error(f"Failed to change model: {self.response.message}")

        end_time = time.time()
        self.get_logger().info(f"time from request to response: {round(end_time - start_time, 3)} seconds")


    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        # Signal the processing thread to exit and wait for it to finish
        self.processing_thread.join(timeout=1)
        self.get_logger().info('Thread stopped.')
        super().destroy_node()

    
    def get_str_param(self, name: str, default: str = "") -> str:
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().string_value
    

    def load_config(self):
        with open(self.config_path, "r") as file:
            self.config = yaml.safe_load(file)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# TODO: subscribe to model node, Need new message that is composed of RGBD + results of segmentation   DONE 
# TODO: handle the new the results field without breaking the abstraction   DONE 
# TODO: need to add client service to select model in model_node   DONE 
    