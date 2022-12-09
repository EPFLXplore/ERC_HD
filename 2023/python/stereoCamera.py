import pyrealsense2 as rs
import numpy as np

class StereoCamera:
    def __init__(self, profile ):
        # TODO: Complete with other stuff
        self.profile = profile
        self.intrisic_camera_matrix, self.coeffs =  self.__init_intrinsics(profile)
        pass

    def __init_intrinsics(profile):
        intrinsics =profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

        fx = intrinsics.fx
        fy = intrinsics.fy
        ppx = intrinsics.ppx
        ppy = intrinsics.ppy
        intrisic_camera_matrix = np.array([[fx, 0, ppx],
                                           [0, fy, ppy],
                                           [0, 0, 1]])

        coeffs = np.array(intrinsics.coeffs)

        return intrisic_camera_matrix, coeffs

    
    def get_intrinsic_camera_matrix(self):
        return self.intrisic_camera_matrix

    def get_coeffs(self):
        return self.get_coeffs