from rclpy.node import Node
from .pipeline_interface import PipelineInterface

from ..modules.instance_segmentation import InstanceSegmentation
from numpy import ndarray
import cv2

#TODO custom msg

class RocksPipeline(PipelineInterface):
    def __init__(self, config_file: str, node: Node, draw_results: bool = True):
        # self.camera_info = {"camera_matrix": camera_matrix, "dist_coeffs": dist_coeffs}   #TODO necessary ?
        super().__init__(config_file, node, draw_results)
        self.config_file = config_file

    def _initialize_publishers(self, node: Node):
        return super()._initialize_publishers(node)
    
    def _initialize_pipeline(self):
        self.instance_segmentation = InstanceSegmentation(self, self.confg_file)
    
    def draw(self, frame: ndarray):
        self.instance_segmentation.draw(frame)
        return frame  #TODO

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
                results = self.instance_segmentation.__call__(rgb_frame=color_frame)

                if results:
                    for result in results:
                        self.draw(result)

                # Exit loop if 'q' is pressed
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
        finally:
            # Stop the pipeline and close OpenCV windows
            pipeline.stop()
            cv2.destroyAllWindows()
