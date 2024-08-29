import cv2
import numpy as np
import os
from ultralytics import YOLO

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

from custom_msg.srv import InitializeModel
from custom_msg.srv import Detect
from custom_msg.msg import BoundingBox
from custom_msg.msg import SegmentationData
from sensor_msgs.msg import CompressedImage


class ModelServer(Node):
    def __init__(self):
        super().__init__("HD_model_server")

        # Initialize YOLO to None
        self.yolo = None

        # Declare 'model_path' parameter
        self.declare_parameter('model_path', '')

        # Servers to initialize the model and perform detection
        self.init_model_srv = self.create_service(InitializeModel, "/HD/model_server/init_model", self.init_model)
        self.detect_srv = self.create_service(Detect, "/HD/model_server/detect", self.detect)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize the YOLO model when the node starts
        self.initialize_model_on_start()

    def initialize_model_on_start(self):
        # Get the 'model_path' parameter
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        if model_path:  # Check if the model path is provided
            # Create a mock request for initializing the model
            request = InitializeModel.Request()
            request.model_path = model_path

            # Create a mock response
            response = InitializeModel.Response()

            # Call the init_model method
            self.init_model(request, response)

            # Check if the model initialized successfully
            if response.success:
                self.get_logger().info("Model initialized successfully on startup with model path: " + model_path)
            else:
                self.get_logger().error("Failed to initialize model on startup.")
        else:
            self.get_logger().error("Model path parameter not set. Cannot initialize model.")

    def init_model(self, request, response):
        # Load YOLO model
        self.yolo = YOLO(request.model_path)
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

        # segmentation_data_list = []  # Initialize a list to store segmentation data

        # # Check if there are any results
        # if results:
        #     for result in results:
        #         masks = result.masks.data.cpu().numpy()  # Assuming masks is a torch.Tensor
        #         boxes = result.boxes.data.cpu().numpy()  # Assuming boxes is a torch.Tensor

        #         for mask, box in zip(masks, boxes):
        #             segmentation_data = SegmentationData()  # Create a new SegmentationData message
                    
        #             # Convert mask to ROS Image message
        #             mask_image_msg = self.bridge.cv2_to_imgmsg(mask.astype(np.uint8), encoding="mono8")
        #             segmentation_data.mask = mask_image_msg

        #             # BoundingBox msg
        #             x1, y1, x2, y2 = map(float, box[:4])  # Extract coordinates
        #             confidence = float(box[4])  # Extract confidence score
        #             class_id = int(box[5])  # Extract class ID
                    
        #             bbox_msg = BoundingBox(x1=x1, y1=y1, x2=x2, y2=y2, confidence=confidence, class_id=class_id)
        #             segmentation_data.box = bbox_msg

        #             # # Convert box to ROS Image message (example: drawing box on blank image)
        #             # box_image = np.zeros_like(cv_image)
        #             # x_min, y_min, x_max, y_max = map(int, box[:4])
        #             # cv2.rectangle(box_image, (x_min, y_min), (x_max, y_max), (255, 255, 255), thickness=2)
        #             # box_image_msg = self.bridge.cv2_to_imgmsg(box_image, encoding="bgr8")
        #             # segmentation_data.box = box_image_msg

        #             # Append the created segmentation data to the list
        #             segmentation_data_list.append(segmentation_data)

        # # Here you would probably want to add this data to your response or process it further.
        # # Assuming your response has a list of SegmentationData to be sent back
        # response.segmentation_data = segmentation_data_list

        response.results = self.bridge.cv2_to_imgmsg(results)  #TODO imgmsg ?
        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ModelServer() 
    rclpy.spin(node)


if __name__ == "__main__":
    main()
