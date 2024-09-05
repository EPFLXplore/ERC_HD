import cv2
import numpy as np
from numpy import ndarray
import os
from ultralytics import YOLO

import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

from custom_msg.msg import BoundingBox
from custom_msg.msg import Mask
from custom_msg.msg import SegmentationData
from custom_msg.msg import CompressedRGBD  # Custom message type
from custom_msg.msg import Model 

from sensor_msgs.msg import CompressedImage

from custom_msg.srv import RequestHDGoal


import time

class ModelNode(Node):
    def __init__(self):
        super().__init__("HD_model_node")
        self._logger.info("Booting .....")

        # Initialize YOLO to None
        self._logger.info("Loading model")
        # self.model = YOLO('./src/perception/models/rock_nael_200.pt')
        self.model = None
        self._logger.info("Segmentation model loaded")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # ROS 2 Subscriptions
        self.rgbd_sub = self.create_subscription(
            CompressedRGBD, "/HD/camera/rgbd", self.rgbd_callback, 10
        )

        # ROS 2 Publishers
        self.processed_rgb_pub = self.create_publisher(
            Model, "/HD/model/image", 1 # TODO publish on the /hd/model/image   DONE
        ) # TODO publish recieved message + results from segmentation    DONE

        # Server (to initialize model)
        self.goal_request_srv = self.create_service(
            RequestHDGoal, self.get_str_param("hd_model_set_goal_srv"), self.set_model
        )

        self._logger.info('Up and Running, come and get segmented :p')

    def get_str_param(self, name: str, default: str = "") -> str:
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().string_value
    
    def set_model(self, request, response):
        self._logger.info("Received model_path request")

        # Load YOLO model using the model path provided in the request
        try:
            if not '/' in request.goal.target:
                self.model = None 
                response.success = True
                self.get_logger().info(f"No model set.  model path: {request.model_path}")

            else:

                self.model = YOLO(request.goal.target)
                response.success = True
                self.get_logger().info(f"Model initialized successfully with model path: {request.model_path}")
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Failed to initialize model with path {request.model_path}: {e}")
        
        return response


    def rgbd_callback(self, rgbd_msg: CompressedRGBD):
        """Producer method to add new data to the queue."""
        self.get_logger().info('rgbd_callback')

        if self.model is None:
            model_msg = Model()
            model_msg.image = rgbd_msg
        else:


            rgb = self.bridge.compressed_imgmsg_to_cv2(rgbd_msg.color)
            # Detect
            detected = self.detect(rgb)

            # Draw
            #self.draw(rgb, detected)
            rgb_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)

            # Publish RGB + segmentation results  
            model_msg = Model()
            model_msg.image = rgbd_msg
            model_msg.segmentation_data = detected

        self.processed_rgb_pub.publish(model_msg)



    def detect(self, rgb: ndarray):
        start_time = time.time()

        # Perform inference on the color image
        results = self.model(rgb)

        segmentation_data = []

        for result in results:
            # Create an instance of SegmentationData
            segment_data = SegmentationData()   
            segment_data.boxes = []
            segment_data.masks = []

            # Extract and populate names using the class ID mapping
            segment_data.names = [result.names[int(class_id)] for class_id in result.boxes.cls]

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

                segment_data.boxes.append(bounding_box)

            # Extract and populate masks
            if result.masks is not None:
                # print(result.masks.xy)
                for xy in result.masks.xy:
                    mask_msg = Mask()
                    mask_msg.mask_pixel = xy.flatten().tolist()  # Convert ndarray to flat list

                    # Flatten the xy coordinates and add them to the mask message
                    # mask_msg.xy = xy  # Assuming xy is a numpy array or list of numpy arrays
                    # self.get_logger().info(xy)

                    segment_data.masks.append(mask_msg)
            
            segmentation_data.append(segment_data)

        # Assign the segmentation data to the response
        end_time = time.time()
        self._logger.info(f'Detection took {round(end_time - start_time, 3)} seconds')

        return segmentation_data
    

    # def draw(self, image, segmentation_data): # TODO draw has to be moved to perception in instance segmentation.
    #     # print(segmentation_data)
        
    #     for segmentation in segmentation_data:
    #         for vertices in segmentation.masks:
    #             vertices = np.array(vertices.mask_pixel).reshape(-1,2)
    #             # cv2.polylines(image,[vertices.astype(np.int32)],True,(0,255,255))
    #             cv2.fillPoly(image, pts=[vertices.astype(np.int32)], color=(255, 0, 0))
    #         for box in segmentation.boxes:
    #             x1, y1, x2, y2 = int(box.x1), int(box.y1), int(box.x2), int(box.y2)
    #             conf = box.confidence
    #             cls = box.class_id

    #             cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Red color box

    #         # Annotate the frame with the class and confidence score
    #             cv2.putText(image, f"{self.model.names[cls]}: {conf:.2f}", (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        

def main(args=None):
    rclpy.init(args=args)
    node = ModelNode() 
    rclpy.spin(node)
    #rclpy.shutdown()

if __name__ == "__main__":
    main()


# TODO add service to load / change model    DONE 