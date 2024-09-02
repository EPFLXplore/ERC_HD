from rclpy.node import Node
from .pipeline_interface import PipelineInterface

from ..modules.instance_segmentation import InstanceSegmentation
from ..modules.module_rocks import ModuleRocks
from numpy import ndarray
import cv2

from custom_msg.msg import RockArray 
from custom_msg.msg import Rock
from custom_msg.msg import SegmentationData

class RocksPipeline(PipelineInterface):
    def __init__(self, config_file: str, node: Node, camera_matrix: ndarray = None, camera_depth_scale: ndarray = None, draw_results: bool = True):
        self.camera_matrix = camera_matrix
        self.camera_depth_scale = camera_depth_scale
        super().__init__(config_file, node, draw_results)


    def _initialize_publishers(self, node: Node):
        self.pose_publisher = node.create_publisher(
            RockArray, "/HD/perception/rock_pose", 10    
        )
    
    def _initialize_pipeline(self, node: Node):
        self.instance_segmentation_module = InstanceSegmentation(self.config, node)
        self.obj_module = ModuleRocks(self.camera_matrix, self.camera_depth_scale)
        self._name = self.config["name"]


    def run_rgbd(self, rgb_image: ndarray, depth_image: ndarray, segmentation_data) -> None:
        # rgb_image is cv2 

        # Perform inference on the color image
        self.instance_segmentation_module(rgb_image, depth_image, segmentation_data)

        # New ROS msg
        msg = RockArray()

        results = self.obj_module(rgb_image, depth_image, segmentation_data)

        for result in results:
            # New Rock
            rock = Rock()
            rock.pose         = result["quaternion"]
            self._logger.info("Quaternion: " + result["quaternion"])
            rock.max_diameter = result["max_dimension_cm"]
            rock.grab_axis    = result["min_dimension_cm"]
            msg.append(rock)

        self._publish(msg)
        

    def draw(self, frame):
        self.obj_module.draw(frame)

    def _publish(self, msg):
        self.pose_publisher.publish(msg)

    def name(self):
        return self._name