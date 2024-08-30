from rclpy.node import Node
from .pipeline_interface import PipelineInterface

from ..modules.instance_segmentation import InstanceSegmentation
from ..modules.module_rocks import ModuleRocks
from numpy import ndarray
import cv2

import pyrealsense2 as rs

from custom_msg.msg import RockArray 
from custom_msg.msg import Rock
from custom_msg.msg import SegmentationData

class RocksPipeline(PipelineInterface):
    def __init__(self, config_file: str, node: Node, draw_results: bool = True):
        # self.camera_info = {"camera_matrix": camera_matrix, "dist_coeffs": dist_coeffs}   #TODO necessary ?
        super().__init__(config_file, node, draw_results)
        # self.config_file = config_file
        # self.node = node 

    def _initialize_publishers(self, node: Node):
        self.pose_publisher = node.create_publisher(
            RockArray, "/HD/perception/rock_pose", 10    
        )
    
    def _initialize_pipeline(self, node: Node):
        self.instance_segmentation = InstanceSegmentation(self.config, node)
        self.obj_module = ModuleRocks()

    def run_rgbd(self, rgb_image: ndarray, depth_image: ndarray) -> None:
        # Perform inference on the color image
        frame, detected_objs = self.instance_segmentation(rgb_image, depth_image)

        # # Draw bounding boxes 
        # self.draw(frame)   # unnecessary bc already done in module_rocks.py 

        # New ROS msg
        msg = RockArray()

        img, results = self.compute_obj(frame, detected_objs)
        for result in results:
            # New Rock
            rock = Rock()
            rock.pose         = result["quaternion"]
            rock.max_diameter = result["max_dimension_cm"]
            rock.grab_axis    = result["min_dimension_cm"]
            msg.append(rock)

        self._publish(msg)

        #self.draw(frame)
        

    def draw(self, frame):
        self.obj_module.draw(frame)
        #self.instance_segmentation.draw(frame)

    def compute_obj(self, frame, detected_objs):
        return self.obj_module.__call__(rgb_frame=frame, detected_objs=detected_objs)  #TODO rgb_frame, depth_frame

    def _publish(self, msg):
        self.pose_publisher.publish(msg)