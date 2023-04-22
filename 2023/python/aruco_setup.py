import cv2 as cv
from cv2 import aruco

from buttons import (button_centers_0,
                     button_centers_1,
                     button_centers_2, 
                     button_centers_3)

marker_dict_4 = aruco.Dictionary_get(aruco.DICT_4X4_100)
marker_dict_5 = aruco.Dictionary_get(aruco.DICT_5X5_100)

param_markers = aruco.DetectorParameters_create()

MARKER_REAL_SIZE = [3.2, 3.2, 4.9, 4.9] #centimeters
TARGET_POINT = [button_centers_0(), button_centers_1(), button_centers_2(), button_centers_3() ]#np.array([[7., 0, 0]])

def detect_markers(gray_img, selected_tag, cam_matrix, coeffs):
    marker_corners, marker_IDs, rej = aruco.detectMarkers(gray_img, marker_dict_4, parameters=param_markers)

    if marker_corners:
        # get rotation and translation vectors
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_REAL_SIZE[selected_tag], cam_matrix, coeffs)

        for i, (ids, corners)  in enumerate(zip(marker_IDs, marker_corners)):
            if ids == selected_tag:
                corners = corners.reshape(4, 2).astype(int)
                top_right, top_left, bottom_right, bottom_left = corners

                [image_points, jacobian] = cv.projectPoints(TARGET_POINT[selected_tag], rVec[i], tVec[i], cam_matrix, coeffs)
                return corners, ids[0], image_points, rVec[i], tVec[i]

    return None, None, None, None, None