import cv2
import numpy as np
from numpy import ndarray
import os
import pyrealsense2 as rs

# from ultralytics import YOLO

from .module_interface import ModuleInterface


class InstanceSegmentation(ModuleInterface):
    # confg_file conatins path to model
    def __init__(self, confg_file):
        # Service client

        # Load the YOLOv8 segmentation model
        # self.model = YOLO(confg_file)

        # Create a pipeline for RealSense
        # self.pipeline = rs.pipeline()

        # # Create a config and configure the pipeline to stream color frames
        # self.config = rs.config()
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pass

    def get_model(self):
        return self.model

    def get_pipeline(self):
        return self.pipeline

    def get_config(self):
        return self.config

    def __call__(self, rgb_frame: ndarray, depth_frame: ndarray):
        # Call service for model inference

        # # Start the pipeline
        # self.pipeline.start(self.config)

        # try:
        #     # Create a window to display the livestream
        #     cv2.namedWindow("YOLOv8 Segmentation on RealSense", cv2.WINDOW_AUTOSIZE)

        #     # img_count = 0  # Image counter for naming the files

        #     while True:
        #         # Wait for a coherent pair of frames: color frame
        #         frames = self.pipeline.wait_for_frames()
        #         self.color_frame = frames.get_color_frame()

        #         if not self.color_frame:
        #             continue

        #         # Perform inference on the color image
        #         results = self.model(self.color_frame)
        #         if results:
        #             for result in results:
        #                 self.draw(result)

        #         # Exit loop if 'q' is pressed
        #         key = cv2.waitKey(1) & 0xFF
        #         if key == ord('q'):
        #             break

        # finally:
        #     # Stop the pipeline and close OpenCV windows
        #     self.pipeline.stop()
        #     cv2.destroyAllWindows()

        return self.model(rgb_frame)

    # frame: result of inference on color img
    def draw(self, frame: ndarray):
        # Check if there are any results
        masks = frame.masks  # Extract the masks
        boxes = frame.boxes  # Extract the bounding boxes

        # Check if masks are not None
        if masks is not None:
            # Convert the masks to a binary format and apply them to the frame
            for mask in masks.data:  # Iterate over the individual masks
                mask = mask.cpu().numpy()  # Convert the mask to a NumPy array
                mask = mask.squeeze()  # Remove unnecessary dimensions
                mask = (mask > 0.5).astype(np.uint8)  # Convert to binary mask

                # Create a 3-channel mask for color blending
                mask_img = np.zeros_like(self.color_frame, dtype=np.uint8)
                mask_img[:, :, 1] = mask * 255  # Apply green color to the mask

                # Blend the mask with the original frame
                self.color_frame = cv2.addWeighted(
                    self.color_frame, 1, mask_img, 0.5, 0
                )

        # Check if boxes are not None
        if boxes is not None:
            # Draw bounding boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                cls = box.cls[0]
                # Draw the bounding box on the frame
                cv2.rectangle(
                    self.color_frame, (x1, y1), (x2, y2), (0, 0, 255), 2
                )  # Red color box
                # Annotate the frame with the class and confidence score
                cv2.putText(
                    self.color_frame,
                    f"{self.model.names[int(cls)]}: {conf:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )
