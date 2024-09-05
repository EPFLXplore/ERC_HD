from rclpy.node import Node
from .pipeline_interface import PipelineInterface

from ..modules.instance_segmentation import InstanceSegmentation
from ..modules.module_rocks import ModuleRocks
from numpy import ndarray
import cv2

from custom_msg.msg import RockArray 
from custom_msg.msg import Rock
from custom_msg.msg import SegmentationData

from geometry_msgs.msg import Pose, Point, Quaternion


class RocksPipeline(PipelineInterface):
    def __init__(self, config_file: str, node: Node, camera_matrix: ndarray = None, camera_depth_scale: ndarray = None, draw_results: bool = True):
        self.camera_matrix = camera_matrix
        self.camera_depth_scale = camera_depth_scale

        # node.get_logger().info(f"Fx: {camera_matrix[0][0]}, fy: {camera_matrix[1][1]}")
        # node.get_logger().info(f"Depth scale: {camera_depth_scale}")

        super().__init__(config_file, node, draw_results)
        self.node = node  # TODO temp for logging


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

        temp = []

        results, target_idx = self.obj_module(rgb_image, depth_image, segmentation_data)

        self.node._logger.info(f"Target idx: {target_idx}")

        for result in results:

            rock_center_depth = result["rock_center_depth"]
            rock_depth = result["depth_surface"]
            self.node._logger.info(f"Rock center depth: {rock_center_depth}")
            self.node._logger.info(f"Depth: {rock_depth}")

            rock_center = result["center"]
            self.node._logger.info(f"Rock center : {rock_center}")

            # New Rock
            rock = Rock()

            rock.center = Point()
            rock.center.x = result["rock_center_coordinates"][0]
            rock.center.y = result["rock_center_coordinates"][1]
            rock.center.z = result["rock_center_coordinates"][2]
            rock.height = 2*(result["rock_center_depth"]-result["depth_surface"])
            rock.angle  = result["angle"] 
            rock.max_diameter = float(result["max_dimension_cm"])
            rock.min_diameter = float(result["min_dimension_cm"])
            temp.append(rock)

        msg.rocks = temp 
        msg.target_index = target_idx
        self._publish(msg)
        

    def draw(self, frame):
        self.obj_module.draw(frame)

    def _publish(self, msg):
        self.pose_publisher.publish(msg)

    def name(self):
        return self._name