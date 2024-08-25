from ..geometry import translation_rotation
from ..handlers.pose_msg import PoseMsg
from .pipeline_interface import PipelineInterface

from numpy import ndarray
from rclpy.node import Node

from geometry_msgs.msg import TargetInstruction

from ..modules.aruco_detector import ArucoDetector

from ..controlpanel import control_panel


class ButtonsPipeline(PipelineInterface):
    def __init__(
        self,
        config_file: str,
        node: Node,
        draw_results: bool = True,
        camera_matrix: ndarray = None,
        dist_coeffs: ndarray = None,
    ):
        self.camera_info = {"camera_matrix": camera_matrix, "dist_coeffs": dist_coeffs}
        super().__init__(config_file, node, draw_results)
        self.control_panel = control_panel.ControlPanel(
            **self.camera_info, button_values=[x for x in range(10)]
        )

    def run_rgb(self, image):
        rvec, tvec = self.aruco_detector.process_rgb(image)
        if rvec is not None and tvec is not None:
            return translation_rotation([0, 0, 0], rvec, tvec)

    def run_rgbd(self, rgb_image: ndarray, depth_image: ndarray) -> None:
        self.run_rgb(rgb_image)

    def _initialize_publishers(self, node: Node):
        self.pose_publisher = node.create_publisher(
            TargetInstruction, "/HD/perception/button_pose", 10
        )

    def _initialize_pipeline(self):
        self.aruco_detector = ArucoDetector(**self.config, **self.camera_info)

    def draw(self, frame: ndarray):
        pass
