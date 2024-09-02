import threading
import queue
from time import sleep
import cv2
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage  # Image is the message type
from custom_msg.msg import CompressedRGBD  # Custom message type
import numpy as np
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from custom_msg.srv import CameraParams, ButtonPressControlPanel
import re

from .pipelines.pipeline_interface import PipelineInterface
from .pipelines.pipeline_factory import PipelineFactory

class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        # used to match button service requests to distinguish them from switching pipeline request
        self.button_pattern = r"^\d[ud]$"  

        # Initialize Camera Info
        self.camera_matrix = None
        self.dist_coeffs = None
        self._get_camera_params()

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize Pipeline 
        self.pipeline: PipelineInterface = PipelineFactory.create_pipeline(
            "rocks", self
        )

        # Initialize Queue for Producer-Consumer model
        self.queue = queue.Queue(maxsize=10)

        # ROS 2 Subscriptions
        self.rgbd_sub = self.create_subscription(
            CompressedRGBD, "/HD/camera/rgbd", self.rgbd_callback, 10
        )

        # ROS 2 Publishers
        self.processed_rgb_pub = self.create_publisher(
            CompressedImage, "/hd/perception/image", 1
        )

        # ROS 2 Services
        self.srv = self.create_service(
            ButtonPressControlPanel,
            "/gui_node/button_press_service",
            self.handle_button_press,
        )

        

        # Start consumer thread for pipeline processing
        self.pipeline_thread = threading.Thread(target=self.call_pipeline)
        self.pipeline_thread.start()

        self.get_logger().info(f"Perception Node started with {self.pipeline.name()} pipeline")

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
        self.camera_matrix = np.array(
            [
                [camera_params.fx, 0, camera_params.cx],
                [0, camera_params.fy, camera_params.cy],
                [0, 0, 1],
            ]
        )
        self.dist_coeffs = np.array(camera_params.distortion_coefficients)

    def rgbd_callback(self, rgbd_msg: CompressedRGBD):
        """Producer: Receives images and adds them to the queue."""
        try:
            # Convert compressed image to OpenCV format
            self.get_logger().info('rgbd_callback')
            rgb = self.bridge.compressed_imgmsg_to_cv2(rgbd_msg.color)
            depth_image = self.bridge.imgmsg_to_cv2(rgbd_msg.depth, "mono16")

            # Add the image to the queue
            self.image_queue.put((rgb, depth_image), timeout=1)  # Timeout to prevent blocking indefinitely
            self.get_logger().info('Received and queued RGBD image')
        except queue.Full:
            self.get_logger().warning('Image queue is full, dropping frame')

    def call_pipeline(self):
        """Consumer method to process data from the queue."""
        while rclpy.ok():
            try:
                # Retrieve data from the queue
                rgb, depth_image = self.queue.get(timeout=1)  # Wait for up to 1 second for new data
                self.get_logger().info('Run through pipeline: START')
                self.pipeline.run_rgbd(rgb, depth_image)
                self.get_logger().info('Run through pipeline: END')

                # Convert the processed image back to ROS message and publish
                rgb_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)
                self.processed_rgb_pub.publish(rgb_msg)
                self.get_logger().info('Published annotated image')
                
                # Indicate that the queue task is complete
                self.queue.task_done()

                

            except queue.Empty:
                self.get_logger().info('Waiting for new images...')

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

        response.response = f"Button {request.button_name} was pressed"
        return response

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        # Signal the processing thread to exit and wait for it to finish
        self.processing_thread.join(timeout=1)
        self.get_logger().info('Thread stopped.')
        super().destroy_node()

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
