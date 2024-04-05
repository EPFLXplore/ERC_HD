import pyrealsense2 as rs
import numpy as np
import cv2 as cv


class StereoCamera:
    def __init__(self):
        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.

        # TODO: Add a configuration object for the pipeline.
        config = rs.config()

        res = {"x": 640, "y": 480}
        # res = {"x": 1280, "y": 720}

        FPS = 15

        config.enable_stream(rs.stream.depth, res["x"], res["y"], rs.format.z16, FPS)
        config.enable_stream(rs.stream.color, res["x"], res["y"], rs.format.rgb8, FPS)

        # Start streaming from file
        self.profile = self.pipe.start(config)

        # self.profile = self.pipe.start().get_stream(
        #     rs.stream.depth
        # )  # Start the pipeline and get the depth stream profile.
        self.intrisic_camera_matrix, self.coeffs = self.__init_intrinsics(
            self.profile
        )  # Initialize the intrinsic camera matrix and distortion coefficients.

    # A private method to initialize the intrinsic camera matrix and distortion coefficients.
    def __init_intrinsics(self, profile):
        # intrinsics = (
        #     profile.as_video_stream_profile().get_intrinsics()
        # )  # Get the intrinsics of the stream profile.

        intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        # Extract the necessary parameters from the intrinsics object.
        fx = intrinsics.fx
        fy = intrinsics.fy
        ppx = intrinsics.ppx
        ppy = intrinsics.ppy

        # Create the intrinsic camera matrix using the extracted parameters.
        intrisic_camera_matrix = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])

        # Extract the distortion coefficients from the intrinsics object.
        coeffs = np.array(intrinsics.coeffs)

        return intrisic_camera_matrix, coeffs

    # A method to get the intrinsic camera matrix.
    def get_intrinsics(self):
        return self.intrisic_camera_matrix

    # A method to get the distortion coefficients.
    def get_coeffs(self):
        return self.coeffs

    # A method to get the depth data from the RealSense camera.
    def get_depth(self):
        frameset = (
            self.pipe.wait_for_frames()
        )  # Wait for the next set of frames from the pipeline.
        depth_frame = (
            frameset.get_depth_frame()
        )  # Get the depth frame from the frameset.

        depth = np.asanyarray(
            depth_frame.get_data()
        )  # Convert the depth frame to a NumPy array.

        return depth

    # A method to get the color image from the RealSense camera.
    def get_image(self):
        frameset = (
            self.pipe.wait_for_frames()
        )  # Wait for the next set of frames from the pipeline.
        color_frame = (
            frameset.get_color_frame()
        )  # Get the color frame from the frameset.

        color = np.asanyarray(
            color_frame.get_data()
        )  # Convert the color frame to a NumPy array.
        frame = cv.cvtColor(
            color, cv.COLOR_BGR2RGB
        )  # Convert the color space of the image.

        return frame
