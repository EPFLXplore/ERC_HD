from .monocular_camera_interface import (
    MonocularCameraInterface,
)
from abc import abstractmethod


class StereoCameraInterface(MonocularCameraInterface):
    @abstractmethod
    def get_depth(self):
        pass

    @abstractmethod
    def get_intrinsics(self):
        pass

    @abstractmethod
    def get_coeffs(self):
        pass

    @abstractmethod
    def get_rgbd(self):
        pass
