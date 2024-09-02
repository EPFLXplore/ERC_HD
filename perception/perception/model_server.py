import cv2
import numpy as np
import os
from ultralytics import YOLO

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

from custom_msg.msg import BoundingBox
from custom_msg.msg import Mask
from custom_msg.msg import SegmentationData

from custom_msg.srv import InitializeModel
from custom_msg.srv import Detect

import time

class ModelServer(Node):
    def __init__(self):
        super().__init__("HD_model_server")
        self._logger.info("Booting .....")


        # Initialize YOLO to None
        self._logger.info("Loading model")
        self.model = YOLO('./src/perception/models/brick_n_200.pt')
        self._logger.info("Segmentation model loaded")

        # Declare 'model_path' parameter
        self.declare_parameter('model_path', '')

        # Servers to initialize the model and perform detection
        self.init_model_srv = self.create_service(InitializeModel, "/HD/model_server/init_model", self.init_model)
        self.detect_srv = self.create_service(Detect, "/HD/model_server/detect", self.detect)
        

        # Initialize CvBridge
        self.bridge = CvBridge()

        self._logger.info('Up and Running, come and get segmented :p')

        # Initialize the YOLO model when the node starts
        #self.initialize_model_on_start()

    # def initialize_model_on_start(self):
    #     # Get the 'model_path' parameter
    #     model_path = self.get_parameter('model_path').get_parameter_value().string_value

    #     if model_path:  # Check if the model path is provided
    #         # Create a mock request for initializing the model
    #         request = InitializeModel.Request()
    #         request.model_path = model_path

    #         # Create a mock response
    #         response = InitializeModel.Response()

    #         # Call the init_model method
    #         self.init_model(request, response)

    #         # Check if the model initialized successfully
    #         if response.success:
    #             self.get_logger().info("Model initialized successfully on startup with model path: " + model_path)
    #         else:
    #             self.get_logger().error("Failed to initialize model on startup.")
    #     else:
    #         self.get_logger().error("Model path parameter not set. Cannot initialize model.")

    # def init_model(self, request, response):
    #     # Load YOLO model
    #     self.yolo = YOLO(request.model_path)

    #     response.success = True
    #     self._logger.info("Server initialized")
    #     return response
    
    def init_model(self, request, response):
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self._logger.info("Received model_path request")

        # Load YOLO model using the model path provided in the request
        try:
            self.model = YOLO(request.model_path)
            response.success = True
            self.get_logger().info(f"Model initialized successfully with model path: {request.model_path}")
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Failed to initialize model with path {request.model_path}: {e}")
        
        return response


    def detect(self, request, response):
        start_time = time.time()
        self._logger.info("Received segmentation request")

        # Check if the model is initialized
        if self.model is None:
            response.success = False
            response.message = "Model not initialized"
            return response
        
        if len(request.image.data) == 0:
            raise ValueError("CompressedImage data is empty.")

        # Convert ROS message to OpenCV image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(request.image)

        # Perform inference on the color image
        results = self.model(cv_image)

        segmentation_data = []

        for result in results:
            # Create an instance of SegmentationData
            segment_data = SegmentationData()   

            # Extract and populate names using the class ID mapping
            # segment_data.names = [results.names[int(class_id)] for class_id in result.boxes.cls]

            # Extract bounding boxes, confidence scores, class labels, and tracking IDs
            for i in range(len(result.boxes.xyxy)):
                bounding_box = BoundingBox()
                bounding_box.x1 = float(result.boxes.xyxy[i][0])  # Top-left x coordinate
                bounding_box.y1 = float(result.boxes.xyxy[i][1])  # Top-left y coordinate
                bounding_box.x2 = float(result.boxes.xyxy[i][2])  # Bottom-right x coordinate
                bounding_box.y2 = float(result.boxes.xyxy[i][3])  # Bottom-right y coordinate
                bounding_box.confidence = float(result.boxes.conf[i])  # Confidence score of the detection
                bounding_box.class_id = int(result.boxes.cls[i])  # Class ID of the detected object

                # Check if tracking IDs are included
                if result.boxes.is_track and result.boxes.id is not None:
                    bounding_box.track_id = int(result.boxes.id[i])  # Tracking ID
                else:
                    bounding_box.track_id = -1  # Default value if tracking ID is not available

                # segment_data.boxes.append(bounding_box)

            # Extract and populate masks
            if result.masks is not None:
                for mask_data, xy in zip(result.masks.data, result.masks.xy):
                    mask_msg = Mask()
                    mask_msg.mask_pixel = xy.flatten().tolist()  # Convert ndarray to flat list
                    # mask_msg.data = mask_data.cpu().numpy()  # Assuming mask_data is a numpy array
                    # mask_msg.width = results.masks.orig_shape[1]  # Width of the mask
                    # mask_msg.height = results.masks.orig_shape[0]  # Height of the mask
                    
                    # Flatten the xy coordinates and add them to the mask message
                    # mask_msg.xy = xy  # Assuming xy is a numpy array or list of numpy arrays
                    # self.get_logger().info(xy)

                    # segment_data.masks.append(mask_msg)
            
            segmentation_data.append(segment_data)

        # Assign the segmentation data to the response
        response.segmentation_data = segmentation_data
        response.success = True

        
        end_time = time.time()
        self._logger.info(f'Detection took {round(end_time - start_time, 3)} seconds, result: {response.success}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModelServer() 
    rclpy.spin(node)
    #rclpy.shutdown()

if __name__ == "__main__":
    main()
