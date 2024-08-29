import cv2
import numpy as np
import os
from ultralytics import YOLO

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

class ModelServer(Node):
    def __init__(self):
        super().__init__("/HD/model_server")

        # Initialize YOLO to None
        self.yolo = None

        # Servers to initialize the model and perform detection
        self.init_model_srv = self.create_service(InitializeModel, "/HD/model_server/init_model", self.init_model)
        self.detect_srv = self.create_service(Detect, "/HD/model_server/detect", self.detect)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def init_model(self, request, response):
        # Load YOLO model
        self.yolo = YOLO(request.model_path, device="cuda")
        response.success = True
        return response

    def detect(self, request, response):

        # Check if the model is initialized
        if self.yolo is None:
            response.success = False
            response.message = "Model not initialized"
            return response

        # Convert ROS message to OpenCV image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(request.image)

        # Perform inference on the color image
        results = self.yolo(cv_image)

        # Check if there are any results
        if results:
            for result in results:
                masks = result.masks  # Extract the masks
                boxes = result.boxes  # Extract the bounding boxes

                # Check if masks are not None
                if masks is not None:
                    # Convert the masks to a binary format and apply them to the frame
                    for mask in masks.data:  # Iterate over the individual masks
                        mask = mask.cpu().numpy()  # Convert the mask to a NumPy array
                        mask = mask.squeeze()  # Remove unnecessary dimensions
                        mask = (mask > 0.5).astype(np.uint8)  # Convert to binary mask

                        # Create a 3-channel mask for color blending
                        mask_img = np.zeros_like(color_image, dtype=np.uint8)
                        mask_img[:, :, 1] = mask * 255  # Apply green color to the mask

                        # Blend the mask with the original frame
                        color_image = cv2.addWeighted(color_image, 1, mask_img, 0.5, 0)

                # Check if boxes are not None
                if boxes is not None:
                    # Draw bounding boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = box.conf[0]
                        cls = box.cls[0]
                        # Draw the bounding box on the frame
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red color box
                        # Annotate the frame with the class and confidence score
                        cv2.putText(color_image, f"{self.model.names[int(cls)]}: {conf:.2f}", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Convert the processed image back to ROS message
        response.image = self.bridge.cv2_to_compressed_imgmsg(color_image)
        response.success = True
        return response