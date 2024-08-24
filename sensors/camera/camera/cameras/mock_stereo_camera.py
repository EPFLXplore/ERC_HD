from ..interfaces.stereo_camera_interface import StereoCameraInterface
import numpy as np


class MockStereoCamera(StereoCameraInterface):
    def __init__(self):
        self.resolution = (640, 480)  # Example resolution
        self.intrinsics = {
            "fx": 600.0,
            "fy": 600.0,
            "cx": 320.0,
            "cy": 240.0,
        }
        self.coeffs = [0.1, -0.05, 0.001, 0.0, 0.0]  # Example distortion coefficients

    def get_image(self):
        # Return a dummy RGB image (e.g., a plain color or random noise)
        return np.random.randint(
            0, 256, (self.resolution[1], self.resolution[0], 3), dtype=np.uint8
        )

    def get_depth(self):
        # Return a dummy depth image (e.g., a gradient or random noise)
        return np.random.randint(
            0, 256, (self.resolution[1], self.resolution[0]), dtype=np.uint16
        )

    def get_intrinsics(self):
        # Return mock intrinsics
        return self.intrinsics

    def get_coeffs(self):
        # Return mock distortion coefficients
        return self.coeffs
