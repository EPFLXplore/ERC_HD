from .cameras.realsense_stereo_camera import RealSenseStereoCamera
from .cameras.mock_monocular_camera import MockMonocularCamera
from .cameras.mock_stereo_camera import MockStereoCamera
from .cameras.monocular_camera import MonocularCamera
from .cameras.oakd_stereo_camera import OakDStereoCamera


class CameraFactory:
    @staticmethod
    def create_camera(camera_type: str):
        if camera_type == "realsense_stereo":
            return RealSenseStereoCamera()
        elif camera_type == "oakd_stereo":
            return OakDStereoCamera()
        elif camera_type == "monocular":
            return MonocularCamera()
        elif camera_type == "mock_stereo":
            return MockStereoCamera()
        elif camera_type == "mock_monocular":
            return MockMonocularCamera()
        else:
            raise ValueError(f"Unknown camera type: {camera_type}")
