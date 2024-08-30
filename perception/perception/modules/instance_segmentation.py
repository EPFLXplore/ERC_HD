import cv2
import numpy as np
from numpy import ndarray
import os
import pyrealsense2 as rs

# from ultralytics import YOLO

import rclpy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

from .module_interface import ModuleInterface

from rclpy.callback_groups import ReentrantCallbackGroup

from custom_msg.srv import Detect
from custom_msg.msg import SegmentationData

class InstanceSegmentation(ModuleInterface):
    # confg_file conatins path to model 
    def __init__(self, confg_file, node):
        reentrant_callback_group = ReentrantCallbackGroup()

        # # Load the YOLOv8 segmentation model
        # self.model = YOLO(confg_file)
        
        # Create a pipeline for RealSense
        self.pipeline = rs.pipeline()

        # Create a pipeline for RealSense
        # self.pipeline = rs.pipeline()

        # Service Client 
        self.camera_service = node.create_client(Detect, "/HD/model_server/detect", 
                                                callback_group=reentrant_callback_group)
        
        self.bridge = CvBridge()  # Initialize CvBridge for conversions between ROS and OpenCV


    # def get_model(self):
    #     return self.model 
    

    def get_pipeline(self):
        return self.pipeline

    def get_config(self):
        return self.config

    def __call__(self, rgb_frame: ndarray, depth_frame: ndarray):
        # Convert rgb_frame (OpenCV image) to ROS CompressedImage message
        rgb_image_msg = self.bridge.cv2_to_compressed_imgmsg(rgb_frame)

        # Create a Detect service request
        detect_request = Detect.Request()
        detect_request.image = rgb_image_msg

        # Call the Detect service
        future = self.camera_service.call_async(detect_request)

        # Wait for the result
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the call was successful
        if future.result() is not None:
            detect_response = future.result()

            if detect_response.success:
                # Process the received segmentation data
                detected_objs = detect_response.results
                return rgb_frame, detected_objs
            else:
                self.get_logger().error(f"Detection failed: {detect_response.message}")
                return None
        else:
            self.get_logger().error("Service call failed")
            return None


    def draw(frame: ndarray) -> None:
        return super().draw()

    # # frame: result of inference on color img 
    # def draw(self, segmentation_data: SegmentationData): 
    #     # Convert the mask and box from SegmentationData to OpenCV images
    #     mask_img = self.bridge.imgmsg_to_cv2(segmentation_data.mask, desired_encoding="mono8")
    #     # box_img = self.bridge.imgmsg_to_cv2(segmentation_data.box, desired_encoding="bgr8")

    #     # Apply the mask to the color frame
    #     mask = (mask_img > 0).astype(np.uint8)  # Convert mask to binary format
    #     # Create a 3-channel mask for color blending
    #     mask_colored = np.zeros_like(self.color_frame, dtype=np.uint8)
    #     mask_colored[:, :, 1] = mask * 255  # Apply green color to the mask

    #     # Blend the mask with the original frame
    #     self.color_frame = cv2.addWeighted(self.color_frame, 1, mask_colored, 0.5, 0)

    #     # Draw bounding boxes
    #     x1, y1, x2, y2 = int(segmentation_data.box.x1), int(segmentation_data.box.y1), int(segmentation_data.box.x2), int(segmentation_data.box.y2)
    #     conf = segmentation_data.box.confidence
    #     cls = segmentation_data.box.class_id

    #     # Draw the bounding box on the frame
    #     cv2.rectangle(self.color_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red color box

    #     # Annotate the frame with the class and confidence score
    #     cv2.putText(self.color_frame, f"{self.model.names[cls]}: {conf:.2f}", (x1, y1 - 10),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        

        # x1, y1, x2, y2 = map(int, box.xyxy[0])
        # conf = box.conf[0]
        # cls = box.cls[0]
        # # Draw the bounding box on the frame
        # cv2.rectangle(self.color_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red color box
        # # Annotate the frame with the class and confidence score
        # cv2.putText(self.color_frame, f"{self.model.names[int(cls)]}: {conf:.2f}", (x1, y1 - 10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
