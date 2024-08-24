from ..interfaces.monocular_camera_interface import MonocularCameraInterface


class MonocularCamera(MonocularCameraInterface):
    def get_image(self):
        # Implementation for monocular camera
        raise NotImplementedError(
            f"get_image not implemented for {self.__class__.__name__}"
        )
