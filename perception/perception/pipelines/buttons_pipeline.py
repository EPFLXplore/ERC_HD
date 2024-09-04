from ..geometry import translation_rotation
from ..handlers.pose_msg import PoseMsg
from .pipeline_interface import PipelineInterface

from numpy import ndarray
from rclpy.node import Node

from custom_msg.msg import ArucoObject

from ..modules.aruco_detector import ArucoDetector


from ..handlers.target_instruction_msg import TargetInstructionMsg
from ..panel.panel import Panel

from ..geometry import get_tranformation_matrix


class ButtonsPipeline(PipelineInterface):
    def __init__(
        self,
        config_file: str,
        node: Node,
        camera_matrix: ndarray = None,
        dist_coeffs: ndarray = None,
        draw_results: bool = True,
    ):
        self.camera_info = {"camera_matrix": camera_matrix, "dist_coeffs": dist_coeffs}
        super().__init__(config_file, node, draw_results)
        self.panel = Panel(config_file, camera_matrix)

    def run_rgb(self, image):
        rvec, tvec = self.aruco_detector.process_rgb(image)

        if rvec is not None and tvec is not None:  # fmt off
            if self.draw_results:  # TODO should be done in the draw function
                self.aruco_detector.draw(image)
                self.panel.draw(image, get_tranformation_matrix(rvec, tvec))

            aruco_translation, aruco_quaternion = translation_rotation(
                [0, 0, 0], rvec, tvec
            )

            target_point = self.panel.get_target()
            print(f"target_point: {target_point}")
            target_translation, target_quaternion = translation_rotation(
                target_point, rvec, tvec
            )
            target_instruction = TargetInstructionMsg.create_message(
                aruco_translation,
                aruco_quaternion,
                target_translation,
                target_quaternion,
                0,
            )
            self.pose_publisher.publish(target_instruction)
        else:
            self.pose_publisher.publish(TargetInstructionMsg.empty_message())

    def run_rgbd(self, rgb_image: ndarray, depth_image: ndarray, unused) -> None:
        self.run_rgb(rgb_image)

    def _initialize_publishers(self, node: Node):
        self.pose_publisher = node.create_publisher(
            ArucoObject, "/HD/perception/button_pose", 10
        )

    def _initialize_pipeline(self, node: Node):

        self.aruco_detector = ArucoDetector(**self.config["aruco"], **self.camera_info)
        self._name = self.config['name']

    def draw(self, frame: ndarray):
        pass

    def set_button(self, button: str):
        self.panel.set_target(button)
        print(f"button pipeline: {button}")
        # self.control_panel.set_button(button)

    def _publish(self):
        pass  # TODO move the publishing from run_rgb to here

    def name(self):
        return self._name