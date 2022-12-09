import cv2 as cv
from cv2 import aruco
import numpy as np

import pyrealsense2 as rs

from buttons import (button_centers_0,
                     button_centers_1,
                     button_centers_2, 
                     button_centers_3)
from stereoCamera import StereoCamera
from utils import select_tag

# Aruco Setup
marker_dict_4 = aruco.Dictionary_get(aruco.DICT_4X4_100)
marker_dict_5 = aruco.Dictionary_get(aruco.DICT_5X5_100)

param_markers = aruco.DetectorParameters_create()

MARKER_REAL_SIZE = [3.2, 3.2, 4.9, 4.9] #centimeters
TARGET_POINT = [button_centers_0(), button_centers_1(), button_centers_2(), button_centers_3() ]#np.array([[7., 0, 0]])

# RealSense Setup
pipe = rs.pipeline()
cfg = rs.config( )  

profile = pipe.start()

profile1 = profile.get_stream(rs.stream.depth)
camera = StereoCamera(profile1)
cam_matrix = camera.get_intrinsic_camera_matrix()
coeffs = camera.get_coeffs()
intr = profile1.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics

selected_tag = 2

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
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_REAL_SIZE[selected_tag], cam_matrix, coeffs)

        for i, (ids, corners)  in enumerate(zip(marker_IDs, marker_corners)):
            if ids == selected_tag:
            
                corners = corners.reshape(4, 2).astype(int)
                top_right, top_left, bottom_right, bottom_left = corners

                [image_points, jacobian] = cv.projectPoints(TARGET_POINT[selected_tag], rVec[i], tVec[i], cam_matrix, coeffs)
     

                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )

                cv.putText(
                    frame, f"id: {ids[0]}", top_right, cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv.LINE_AA,
                )

                # draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_matrix, coeffs, rVec[i], tVec[i], 4, 4)

                for i in range(image_points.shape[0]):
                    cv.circle(frame, (int(image_points[i][0][0]),int(image_points[i][0][1])), 2, (0, 100, 255), 8)
                
                
                target_distance = depth_frame.get_distance(int(image_points[0][0][0]),int(image_points[0][0][1]))
                # print("target distance in m:", target_distance)

    cv.namedWindow('RealSense Depth', cv.WINDOW_AUTOSIZE)
    # cv.imshow('RealSense Color', cv.resize(frame, (1920,1080)))
    cv.imshow('RealSense Color', frame)

    cv.namedWindow('RealSense Depth', cv.WINDOW_AUTOSIZE)
    cv.imshow('RealSense Depth', depth)

    key = cv.waitKey(1)
    print(key)
    if  key == ord('q'):
        break
    else:
        selected_tag = select_tag(key, selected_tag)
    
cv.destroyAllWindows()