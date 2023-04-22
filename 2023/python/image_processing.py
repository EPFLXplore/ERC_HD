import cv2 as cv
import numpy as np

from aruco_setup import detect_markers
from realsense_setup import get_frames

selected_tag = 2

while True:
    frame, depth_frame, gray_img = get_frames()

    corners, ids, image_points, rVec, tVec = detect_markers(gray_img, selected_tag, cam_matrix, coeffs)

    if corners is not None:
        cv.polylines(
            frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
        )