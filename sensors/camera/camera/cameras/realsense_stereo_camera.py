import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import os
import yaml

from ..interfaces.stereo_camera_interface import StereoCameraInterface


class RealSenseStereoCamera(StereoCameraInterface):
    def __init__(self):
        self.config_path = f"{os.getcwd()}/src/sensors/camera/configs/realsense_config.yaml"
        self.load_config()
        

        self.pipe = rs.pipeline()  # Create a RealSense pipeline object.

        # TODO: Add a configuration object for the pipeline.
        config = rs.config()

        # res = {"x": 640, "y": 480}
        # res = {"x": 1280, "y": 720}
        # res_col = {'x': 1920, 'y':1080}
        # res_depth = {'x':1280,'y':720}
        self.fps = self.config['fps']
        # self.using_depth = self.config['use_depth']

        config.enable_stream(rs.stream.depth, self.config['x'], self.config['y'], rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.config['x'], self.config['y'], rs.format.bgr8, self.fps)

        # Start streaming from file
        self.profile = self.pipe.start(config)

        self.align = rs.align(rs.stream.color) #TODO added

    def load_config(self):
        with open(self.config_path, "r") as file:
            conf = yaml.safe_load(file)
        config = conf['x_y_fps']
        self.config = {}
        self.config['x'] = int(config[0])
        self.config['y'] = int(config[1])
        self.config['fps'] = int(config[2])
        # if len(config) < 4:
        #     self.config['use_depth'] = True
        # else:
        #     self.config['use_depth'] = bool(config[3])
        

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

        return color

    def get_rgbd(self):
        frameset = self.pipe.wait_for_frames()
        color_frame = np.asanyarray((frameset.get_color_frame().get_data()))
        depth_frame = np.asanyarray(frameset.get_depth_frame().get_data())
        # depth_frame = self.get_depth()
        return color_frame, depth_frame
