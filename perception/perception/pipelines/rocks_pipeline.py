from rclpy.node import Node
from .pipeline_interface import PipelineInterface

from ..modules.instance_segmentation import InstanceSegmentation
from ..modules.instance_segmentation import ModuleRocks
from numpy import ndarray
import cv2

from custom_msg.msg import RockArray 
from custom_msg.msg import Rock
from custom_msg.msg import SegmentationData

class RocksPipeline(PipelineInterface):
    def __init__(self, config_file: str, node: Node, draw_results: bool = True):
        # self.camera_info = {"camera_matrix": camera_matrix, "dist_coeffs": dist_coeffs}   #TODO necessary ?
        super().__init__(config_file, node, draw_results)
        self.config_file = config_file
        self.node = node 

    def _initialize_publishers(self, node: Node):
        self.pose_publisher = node.create_publisher(
            RockArray, "/HD/perception/rock_pose", 10    
        )
    
    def _initialize_pipeline(self):
        self.instance_segmentation = InstanceSegmentation(self, self.confg_file, self.node)
        self.obj_module = ModuleRocks(self)
    
    def draw(self, frame):
        self.instance_segmentation.draw(frame)

    def run_segmentation(self):
        # Start the pipeline
        pipeline = self.instance_segmentation.get_pipeline
        pipeline.start(self.instance_segmentation.get_config)

        try:
            # # Create a window to display the livestream
            # cv2.namedWindow("YOLOv8 Segmentation on RealSense", cv2.WINDOW_AUTOSIZE)

            # img_count = 0  # Image counter for naming the files
            
            while True:
                # Wait for a coherent pair of frames: color frame
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    continue

                # Perform inference on the color image
                frame, detected_objs = self.instance_segmentation.__call__(rgb_frame=color_frame)

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

                # Exit loop if 'q' is pressed
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
        finally:
            # Stop the pipeline and close OpenCV windows
            pipeline.stop()
            cv2.destroyAllWindows()

    def compute_obj(self, frame, detected_objs):
        return self.obj_module.__call__(rgb_frame=frame, detected_objs=detected_objs)  #TODO rgb_frame, depth_frame

    def _publish(self, msg):
        self.pose_publisher.publish(msg)