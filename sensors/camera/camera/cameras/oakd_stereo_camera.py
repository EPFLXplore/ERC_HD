from ..interfaces.stereo_camera_interface import StereoCameraInterface


class OakDStereoCamera(StereoCameraInterface):
    def __init__(self):
        # Initialize OAK-D camera
        # raise NotImplementedError(f"__init__ not implemented for {self._name}")
        raise NotImplementedError(
            f"__init__ not implemented for {self.__class__.__name__}"
        )

    def get_image(self):
        # Implementation for getting RGB image from OAK-D
        raise NotImplementedError(
            f"get_image not implemented for {self.__class__.__name__}"
        )

    def get_depth(self):
        # Implementation for getting depth data from OAK-D
        raise NotImplementedError(
            f"get_depth not implemented for {self.__class__.__name__}"
        )

    def get_intrinsics(self):
        # Implementation for getting camera intrinsics from OAK-D
        raise NotImplementedError(
            f"get_intrinsics not implemented for {self.__class__.__name__}"
        )

    def get_coeffs(self):
        # Implementation for getting distortion coefficients from OAK-D
        raise NotImplementedError(
            f"get_coeffs not implemented for {self.__class__.__name__}"
        )


# =============================================================================================


def main():
    camera = OakDStereoCamera()
    camera.get_image()


# Import my_module and call `my_main` with args as positional arguments
if __name__ == "__main__":
    main()
