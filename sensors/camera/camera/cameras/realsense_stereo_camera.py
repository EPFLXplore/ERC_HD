import pyrealsense2 as rs
import numpy as np
import cv2 as cv

from ..interfaces.stereo_camera_interface import StereoCameraInterface


class RealSenseStereoCamera(StereoCameraInterface):
    def __init__(self):
        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.

        # TODO: Add a configuration object for the pipeline.
        config = rs.config()

        res = {"x": 640, "y": 480}
        # res = {"x": 1280, "y": 720}
        # res = {"x": 848, "y": 480}
        res_depth = res
        res_col = res
        # res_col = {'x': 1920, 'y':1080}
        # res_depth = {'x':1280,'y':720}
        FPS = 30

        config.enable_stream(rs.stream.depth, res_depth["x"], res_depth["y"], rs.format.z16, FPS)
        config.enable_stream(rs.stream.color, res_col["x"], res_col["y"], rs.format.bgr8, FPS)

        # Start streaming from file
        self.profile = self.pipe.start(config)

        self.align = rs.align(rs.stream.color) #TODO added

    def get_depth_scale(self):
        depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        return depth_scale

    # A method to get the intrinsic camera matrix.
    def get_intrinsics(self):
        intrinsics = (
            self.profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        intrinsics = {
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "cx": intrinsics.ppx,
            "cy": intrinsics.ppy,
        }
        return intrinsics

    # A method to get the distortion coefficients.
    def get_coeffs(self):
        intrinsics = (
            self.profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        # Extract the distortion coefficients from the intrinsics object.
        return intrinsics.coeffs

    # A method to get the depth data from the RealSense camera.
    def get_depth(self):
        frameset = (
            self.pipe.wait_for_frames()
        )  # Wait for the next set of frames from the pipeline.
        aligned_frame = self.align.process(frameset) # TODO added
        depth_frame = (
            aligned_frame.get_depth_frame()
        ) 
        # depth_frame = (
        #     frameset.get_depth_frame()
        # )  # Get the depth frame from the frameset.
        depth = np.asanyarray(
            depth_frame.get_data()
        )  # Convert the depth frame to a NumPy array.

        return depth

    # A method to get the color image from the RealSense camera.
    def get_image(self):
        frameset = self.pipe.wait_for_frames()

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

    def get_rgbd(self):
        frameset = self.pipe.wait_for_frames()
        color_frame = np.asanyarray((frameset.get_color_frame().get_data()))
        # depth_frame = np.asanyarray(frameset.get_depth_frame().get_data())
        depth_frame = self.get_depth()
        return color_frame, depth_frame
