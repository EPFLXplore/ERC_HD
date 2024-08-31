import cv2
import numpy as np
import os
from ultralytics import YOLO

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

from custom_msg.srv import InitializeModel
from custom_msg.srv import Detect
from sensor_msgs.msg import CompressedImage

class ModelServer(Node):
    def __init__(self):
        super().__init__("HD_model_server")
        self._logger.info("Booting .....")


        # Initialize YOLO to None
        self._logger.info("Loading model")
        self.model = YOLO('./src/perception/models/brick_n_200.pt')
        self._logger.info("Segmentation model loaded")

        # Declare 'model_path' parameter
        # self.declare_parameter('model_path', '')

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
        self._logger.info("Received segmentation request")
        # Check if the model is initialized
        # if self.yolo is None:
        #     response.success = False
        #     response.message = "Model not initialized"
        #     return response

        # # Convert ROS message to OpenCV image
        # cv_image = self.bridge.compressed_imgmsg_to_cv2(request.image)

        # self._logger.info("Bridge compressed decoded to img")
        # # Perform inference on the color image
        # results = self.yolo(cv_image)
        # results = cv_image

        # response.image = self.bridge.cv2_to_compressed_imgmsg(results)  #TODO imgmsg ?
        response.image = request.image
        response.success = True

        self._logger.info(f"Server sent response with success: { response.success}")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ModelServer() 
    rclpy.spin(node)


if __name__ == "__main__":
    main()
