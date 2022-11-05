import cv2 as cv
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt

import pyrealsense2 as rs


# Aruco Setup
marker_dict_4 = aruco.Dictionary_get(aruco.DICT_4X4_100)
marker_dict_5 = aruco.Dictionary_get(aruco.DICT_5X5_100)

param_markers = aruco.DetectorParameters_create()

MARKER_REAL_SIZE = 5 #centimeters
TARGET_POINT = np.array([[7., 0, 0]])

# RealSense Setup
pipe = rs.pipeline()
cfg = rs.config()

profile = pipe.start()

profile1 = profile.get_stream(rs.stream.depth)

intr = profile1.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

fx = intr.fx
fy = intr.fy
ppx = intr.ppx
ppy = intr.ppy
coeffs = intr.coeffs
coeffs = np.array(coeffs)
print(len(coeffs))
cam_matrix = np.array([[fx, 0, ppx],
                       [0, fy, ppy],
                       [0, 0, 1]])

print(cam_matrix)

# Skip first 5 frames
for _ in range(5):
    pipe.wait_for_frames()

while True:
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    color = np.asanyarray(color_frame.get_data())
    depth = np.asanyarray(depth_frame.get_data())

    frame = cv.cvtColor(color, cv.COLOR_BGR2RGB)

    gray_img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, rej = aruco.detectMarkers(gray_img, marker_dict_4, parameters=param_markers)

    if marker_corners:

        # get rotation and translation vectors
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_REAL_SIZE, cam_matrix, coeffs)

        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )

            corners = corners.reshape(4, 2)
            corners = corners.astype(int)

            top_right = corners[0]  # .ravel()
            top_left = corners[1]  # .ravel()
            bottom_right = corners[2]  # .ravel()
            bottom_left = corners[3]  # .ravel()

            # draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_matrix, coeffs, rVec[i], tVec[i], 4, 4)
            print(rVec[i])
            print(tVec[i])

            [image_points, jacobian] = cv.projectPoints(TARGET_POINT, rVec[i], tVec[i], cam_matrix, coeffs)

            cv.circle(frame, (int(image_points[0][0][0]),int(image_points[0][0][1])), 4, (0, 100, 255), 8)
            target_distance = depth_frame.get_distance(int(image_points[0][0][0]),int(image_points[0][0][1]))
            print("target distance in m:", target_distance)

    cv.namedWindow('RealSense Depth', cv.WINDOW_AUTOSIZE)
    cv.imshow('RealSense Color', frame)

    cv.namedWindow('RealSense Depth', cv.WINDOW_AUTOSIZE)
    cv.imshow('RealSense Depth', depth)

    cv.waitKey(1)
