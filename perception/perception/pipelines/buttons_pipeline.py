from ..geometry import translation_rotation
from ..handlers.pose_msg import PoseMsg


class ButtonsPipeline:
    def __init__(self, config, aruco_detector):
        self.config = config
        self.aruco_detector = aruco_detector
        self.msg_type = "pose"

    def process_rgb(self, image):
        rvec, tvec = self.aruco_detector.process_rgb(image)
        return translation_rotation([0, 0, 0], rvec, tvec)
