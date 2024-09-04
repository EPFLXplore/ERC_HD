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

        temp = []

        results = self.obj_module(rgb_image, depth_image, segmentation_data)

        for result in results:
            # New Rock
            rock = Rock()

            # rock.pose         = p #result["quaternion"]

            # Instantiate a Pose message
            pose = Pose()

            # Set the position part of the Pose
            pose.position = Point()
            pose.position.x = 1.0
            pose.position.y = 2.0
            pose.position.z = 3.0

            # Set the orientation part of the Pose
            pose.orientation = Quaternion()
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            # self._logger.info("Quaternion: " + result["quaternion"])
            rock.pose = pose 
            rock.max_diameter = float(result["max_dimension_cm"])
            rock.min_diameter = float(result["min_dimension_cm"])
            temp.append(rock)

        msg.rocks = temp 
        self._publish(msg)
        

    def draw(self, frame):
        self.obj_module.draw(frame)

    def _publish(self, msg):
        self.pose_publisher.publish(msg)

    def name(self):
        return self._name