from ..interfaces.monocular_camera_interface import MonocularCameraInterface
import numpy as np


class MockMonocularCamera(MonocularCameraInterface):
    def __init__(self):
        self.resolution = (640, 480)  # Example resolution

    def get_image(self):
        # Return a dummy RGB image (e.g., a plain color or random noise)
        return np.random.randint(
            0, 256, (self.resolution[1], self.resolution[0], 3), dtype=np.uint8
        )
