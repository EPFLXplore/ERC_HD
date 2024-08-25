from rclpy.node import Node
from .pipeline_interface import PipelineInterface

from std_msgs.msg import String
import cv2


class IdentityPipeline(PipelineInterface):

    def __init__(self, config_file: str, node: Node, draw_results: bool = True):
        super().__init__(config_file, node, draw_results)

    def _initialize_pipeline(self):
        self._logger.info("Initializing Identity Pipeline")

    def _initialize_publishers(self, node: Node):
        self._logger.info("Initializing publishers")
        self.identity_pub = node.create_publisher(String, "/hd/perception/identity", 10)

    def run_rgbd(self, rgb_image, depth_image):
        if self.draw_results:
            self.draw(rgb_image)
        self.identity_pub.publish(String(data="identity"))

    def draw(self, frame):

        frame = cv2.putText(
            frame,
            "identity",
            (int(frame.shape[1] / 4), int(frame.shape[0] / 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            5,  # Increase font size to 2
            (0, 255, 0),
            5,
            cv2.LINE_AA,
        )
