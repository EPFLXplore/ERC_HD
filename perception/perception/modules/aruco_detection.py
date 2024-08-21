from cv2 import aruco
import cv2 as cv
import numpy as np
from .perception_module_interface import PerceptionModuleInterface


class ARTag(PerceptionModuleInterface):
    def __init__(
        self,
        marker_dict,
        marker_size,
        marker_ids: list,
        camera_matrix,
        dist_coeffs,
    ):

        # Dictionary is either aruco.DICT_4X4_50 or aruco.DICT_ARUCO_ORIGINAL
        self.marker_dict = aruco.Dictionary_get(marker_dict)
        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.marker_ids = marker_ids

    def process_rgb(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _rejectedImgPoints = aruco.detectMarkers(gray, self.marker_dict)
        if ids in self.marker_ids:
            tag_idx = np.where(ids == self.marker_ids)[0][0]
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            self.rvec = rvecs[tag_idx]
            self.tvec = tvecs[tag_idx]
            self.corners = corners[tag_idx]

    def process_rgbd(self, rgb_frame, depth_frame):
        return self.process_rgb(rgb_frame)

    def __str__(self) -> str:
        return f"ARTag(marker_dict={self.marker_dict},\
                        marker_size={self.marker_size},\
                        camera_matrix={self.camera_matrix},\
                        dist_coeffs={self.dist_coeffs})"

    # Draw the pose of the marker object
    def draw(self, frame, length=50, thickness=4):
        cv.drawFrameAxes(
            frame,
            self.camera_matrix,
            self.dist_coeffs,
            self.rvec,
            self.tvec,
            length,
            thickness,
        )
        cv.polylines(frame, [self.corners.astype(np.int32)], True, (0, 255, 255), 10)

        center = self.corners.mean(axis=1).astype(np.int32)
        cv.circle(frame, tuple(center[0]), 10, (0, 255, 255), -1)

    def project(self, points):
        [projected, jacobian] = cv.projectPoints(
            points, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs
        )
        return projected.reshape((projected.shape[0] // 4, 4, projected.shape[2]))

    def self_project(self):
        self._projected = self.project(self.corners)

    def get_vecs(self):
        return [self.rvec, self.tvec]
