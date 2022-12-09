import cv2 as cv
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt

import pyrealsense2 as rs


def button_centers_0():
    HORIZONTAL_SPACING = 62
    return (np.array([-HORIZONTAL_SPACING, 0., 0.]) / 10).astype(np.float64)


def button_centers_1():
    HORIZONTAL_SPACING = 62
    return (np.array([-HORIZONTAL_SPACING, 0., 0.]) / 10).astype(np.float64)


def button_centers_2():
    HORIZONTAL_SPACING = 85 # in mm 
    VERTICAL_SPACING = 71 # in mm 

    BUTTON_LAYOUT = (2,3)

    button_centers = np.zeros((BUTTON_LAYOUT[0], BUTTON_LAYOUT[1], 3))

    for i in range(BUTTON_LAYOUT[0]):
        for j in range(BUTTON_LAYOUT[1]):
            button_centers[i,j] = np.array([i*HORIZONTAL_SPACING, -j*VERTICAL_SPACING, 0])

    button_centers = button_centers.reshape(-1, *button_centers.shape[-1:])
    print(button_centers.shape)
    return (button_centers / 10).astype(np.float64)

def button_centers_3():
    VERTICAL_SPACING = 71
    return (np.array([-0, VERTICAL_SPACING, 0.]) / 10).astype(np.float64)



def select_tag(key, current):
    OFFSET = 48
    NB_AR_TAGS = 4
    tags_set = set([ord(str(x)) for x in range(NB_AR_TAGS)])

    if key in tags_set:
        return int(key) - OFFSET
    else:
        return current

# Aruco Setup
marker_dict_4 = aruco.Dictionary_get(aruco.DICT_4X4_100)
marker_dict_5 = aruco.Dictionary_get(aruco.DICT_5X5_100)

param_markers = aruco.DetectorParameters_create()

MARKER_REAL_SIZE = [3.2, 3.2, 4.9, 4.9] #centimeters



# SWITCH_CENTERS = 

TARGET_POINT = [button_centers_0(), button_centers_1(), button_centers_2(), button_centers_3() ]#np.array([[7., 0, 0]])

# RealSense Setup
pipe = rs.pipeline()
cfg = rs.config( )  

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
so_far = []

selected_tag = 2

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
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_REAL_SIZE[selected_tag], cam_matrix, coeffs)

        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            if ids == selected_tag:
            #     continue
            
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )

                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                top_right = corners[0]  # .ravel()
                top_left = corners[1]  # .ravel()
                bottom_right = corners[2]  # .ravel()
                bottom_left = corners[3]  # .ravel()

                cv.putText(
                    frame,
                    f"id: {ids[0]}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv.LINE_AA,
                )

                # draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_matrix, coeffs, rVec[i], tVec[i], 4, 4)
                # print(rVec[i])
                # print(tVec[i])

                [image_points, jacobian] = cv.projectPoints(TARGET_POINT[selected_tag], rVec[i], tVec[i], cam_matrix, coeffs)
                # so_far.extend(image_points)
                # image_points = np.array(so_far)

                with open('measures.txt', 'a') as f:
                    f.write(f"{int(image_points[0][0][0])},{int(image_points[0][0][1])}, {int(image_points[5][0][0])}, {int(image_points[5][0][1])}\n")



                # print(len(image_points))
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

