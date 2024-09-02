import time
import cv2
import numpy as np
from numpy import ndarray
import os
import pyrealsense2 as rs

# from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from .module_interface import ModuleInterface

# from rclpy.callback_groups import ReentrantCallbackGroup

# from custom_msg.srv import Detect
# from custom_msg.srv import InitializeModel

from custom_msg.msg import SegmentationData

from rclpy.executors import Executor, Future


class InstanceSegmentation(ModuleInterface):
    # confg_file conatins path to model
    def __init__(self, confg_file, node: Node):
        # reentrant_callback_group = ReentrantCallbackGroup()

        self.node = node

        # # Service Client
        # self.init_client   = node.create_client(InitializeModel, '/HD/model_server/init_model',
        #                                                 callback_group=reentrant_callback_group)
        # self.detect_client = node.create_client(Detect, "/HD/model_server/detect",
        #                                                 callback_group=reentrant_callback_group)

        # # Create a config and configure the pipeline to stream color frames
        # self.config = rs.config()
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # self.node._logger.info('Waiting for InitializeModel and Detect services...')
        # self.init_client.wait_for_service()
        # self.detect_client.wait_for_service()
        # self.node._logger.info('Services are available.')

        self.bridge = (
            CvBridge()
        )  # Initialize CvBridge for conversions between ROS and OpenCV

        # Call the initialize model service
        # self.call_initialize_model_service('/src/perception/models/brick_n_200.pt')

        self.result_segmentation = None

        # https://chatgpt.com/c/1d580ffd-8c66-4fbe-8cc1-8104f335c614

    # def call_initialize_model_service(self, model_path):
    #     request = InitializeModel.Request()
    #     request.model_path = model_path

    #     self.node._logger.info('Sending model_path')

    #     future = self.init_client.call_async(request)

    #     self.node._logger.info('Sent model_path')

    #     rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

    #     if future.done():
    #         if future.result() is not None and future.result().success:
    #             self.node._logger.info('Model initialized successfully.')
    #         else:
    #             self.node._logger.error('Failed to initialize model.')
    #     else:
    #         self.node._logger.error('Timeout waiting for model initialization response.')

    # BELOW :  call() for model_server.py
    # def __call__(self, rgb_frame: ndarray, depth_frame: ndarray):
    #     # Convert rgb_frame (OpenCV image) to ROS CompressedImage message
    #     self.node.get_logger().debug("Start segmentation")
    #     rgb_image_msg = self.bridge.cv2_to_compressed_imgmsg(rgb_frame)

    #     if len(rgb_image_msg.data) == 0:
    #         raise ValueError("CompressedImage data is empty.")

    #     # Create a Detect service request
    #     detect_request = Detect.Request()
    #     detect_request.image = rgb_image_msg

    #     self.node._logger.info("Asking for segmentation")
    #     start_time  = time.time()

    #     # Call the Detect service
    #     self.detect_response = None

    #     future = self.detect_client.call_async(detect_request)
    #     future.add_done_callback(lambda f: self.service_response(f))

    #     self.node._logger.info("Client called")

    #     # Check if the call was successful TODO: create a timeout
    #     while self.detect_response == None:
    #         self.node._logger.debug("Waiting for client")

    #     if self.detect_response.success:
    #         # Process the received segmentation data
    #         detected_objs = self.detect_response.segmentation_data
    #         self.node._logger.info("Success!")
    #         self.result_segmentation = detected_objs
    #     else:
    #         self.get_logger().error(f"Detection failed: {self.detect_response.message}")

    #     end_time = time.time()
    #     self.node._logger.info(f"time from request to response: {round(end_time - start_time, 3)} seconds")

    def __call__(self, rgb_frame: ndarray, depth_frame: ndarray, segmentation_data):
        # rgb_frame is cv2
        self.node.get_logger().debug("Start drawing")
        self.draw(rgb_frame, segmentation_data)

    def service_response(self, f):  # TODO why ?
        self.node._logger.info("Result")
        self.detect_response = f.result()
        self.draw()

    def draw(self, image, segmentation_data):
        for segmentation in segmentation_data:
            for vertices in segmentation.masks:
                vertices = np.array(vertices.mask_pixel).reshape(-1, 2)
                # cv2.polylines(image,[vertices.astype(np.int32)],True,(0,255,255))
                cv2.fillPoly(image, pts=[vertices.astype(np.int32)], color=(255, 0, 0))
            for box in segmentation.boxes:
                x1, y1, x2, y2 = int(box.x1), int(box.y1), int(box.x2), int(box.y2)
                conf = box.confidence
                cls = box.class_id

                cv2.rectangle(
                    image, (x1, y1), (x2, y2), (0, 0, 255), 2
                )  # Red color box

                # Annotate the frame with the class and confidence score
                cv2.putText(
                    image,
                    f"{segmentation.names[cls]}: {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )


# TODO draw here with results from instance segmentation  DONE
