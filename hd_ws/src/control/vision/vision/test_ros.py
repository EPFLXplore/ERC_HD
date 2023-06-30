import numpy as np
import cv2 as cv
from cv2 import aruco
try:
    import pyrealsense2.pyrealsense2 as rs
except:
    import pyrealsense2 as rs
from vision.camera_projection import camera_projection
from scipy.spatial.transform import Rotation as R
from vision.vision_publisher import VisionPublisher

import rclpy
from rclpy.node import Node

from interfaces.msg import PanelObject

from std_msgs.msg import String
import geometry_msgs
from geometry_msgs.msg import Pose

import threading
import numpy.linalg as linalg


def rvec2quat(rVec):
    # conversion to rotation matrix
    r = R.from_rotvec(rVec)
    rot_mat = r.as_dcm()#as_matrix()

    # reshape to 3*3
    rot_mat = rot_mat.T.reshape((3,3))

    # change of reference
    rot_mat = linalg.inv(rot_mat)

    # convert to quat
    r2 = R.from_dcm(rot_mat)#from_matrix(rot_mat)
    quat = r2.as_quat()
    
    return quat


def main(args=None):
    rclpy.init(args=args)

    # aruco setup
    marker_dict_4 = aruco.Dictionary_get(aruco.DICT_4X4_50)
    param_markers = aruco.DetectorParameters_create()
    MARKER_REAL_SIZE = 4.5 #centimeters

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

    cam_matrix = np.array([[fx, 0, ppx],
                        [0, fy, ppy],
                        [0, 0, 1]])



    TARGET_AR = np.array([0., 0., 0.])
    EXPECTED_CAM= [1., 0., 0. ]


    ############
    # PUBLISHER
    ############
    publisher = VisionPublisher()

    threading.Thread(target=rclpy.spin, args=(publisher,), daemon=True).start()

    # Skip first 5 frames
    for _ in range(5):
        pipe.wait_for_frames()

    rate = publisher.create_rate(30)  # 25hz
    
    while True:
        # msg = publisher.create_panelobject_message(0, 1., 2., 3., 0., 0., 0., 0.)
        # publisher.publish_inform(msg)


        frameset = pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        frame = cv.cvtColor(color, cv.COLOR_BGR2RGB)

        gray_img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, rej = aruco.detectMarkers(gray_img, marker_dict_4, parameters=param_markers)
        #print("Camera")
        if marker_corners:
            #print("tags")
            # get rotation and translation vectors
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_REAL_SIZE, cam_matrix, coeffs)

            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):


                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )

                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                # draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_matrix, coeffs, rVec[i], tVec[i], 4, 4)


                [image_points, jacobian] = cv.projectPoints(TARGET_AR, rVec[i], tVec[i], cam_matrix, coeffs)


                cv.circle(frame, (int(image_points[0, 0, 0]), int(image_points[0, 0, 1])), 1, (0, 100, 255), 2)

                point_cam = camera_projection(TARGET_AR, rVec[i], tVec[i])

                quat = rvec2quat(rVec[i])

                msg = publisher.create_panelobject_message(0, 
                                                     *point_cam,
                                                     *quat)
                
                publisher.publish_inform(msg)



        # cv.circle(frame, (frame.shape[1]//2, frame.shape[0]//2), 5, (255,0,0), 2)

        # cv.namedWindow('RealSense Depth', cv.WINDOW_AUTOSIZE)
        # cv.imshow('RealSense Color', frame)

        # cv.namedWindow('RealSense Depth', cv.WINDOW_AUTOSIZE)
        # cv.imshow('RealSense Depth', depth)

        # cv.waitKey(1)

        rate.sleep()
        

if __name__ == '__main__':
    main()

