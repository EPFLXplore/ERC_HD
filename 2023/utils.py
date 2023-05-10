import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R


def show(frame, depth, max_dist = 25500): # everything past 2.55 meters is set to 2.55 meters 
    depth[ depth > max_dist] = max_dist
    depth = 255.0 * depth / depth.max()

    depth = depth.astype(np.uint8)
    depth_3_channel = cv.cvtColor(depth, cv.COLOR_GRAY2BGR)

    numpy_horizontal = np.hstack((frame, depth_3_channel))

    cv.imshow('RealSense', numpy_horizontal) 


"""
    https://www.cse.psu.edu/~rtc12/CSE486/lecture12.pdf slide 19 
"""
def camera_projection(point, rVec, tVec):
    """""
    point: [x, y, z] point to be projected to the camera frame, (x,y,z) are relative to the AR tag's coordinate frame
    rVec: 3x1
    tVec: 3x1
    """""

    # append 1 for homogenous coordinates
    point = np.append(point, [1])

    # compute the rotation matrix
    R, jacobian = cv.Rodrigues(rVec)

    # append extra row and column
    R = np.vstack((R, [0, 0, 0]))
    R = np.hstack((R, [[0], [0], [0], [1]]))

    # identity 4x4
    id_3 = np.eye(3)

    id3 = np.vstack((id_3, [0,0,0]))

    # -tVec appended to identity
    M = np.hstack((id3, [[-tVec[0, 0]], [-tVec[0, 1]], [-tVec[0, 2]], [1]]))

    x, y, z, n = R @ M @ point.T

    return np.array([-x, -y, z]) / n


def next_movement(target, rvec, tvec):
    point_cam = camera_projection(target, rvec, tvec)

    r = R.from_rotvec(rvec)
    rot_mat = r.as_matrix()
    print(rot_mat.shape)
    rot_mat = rot_mat.T.reshape((3,3))
    print(rot_mat.shape)
    r2 = R.from_matrix(rot_mat)
    angles = r2.as_euler('xyz', degrees=True).flatten()

    print("Point in AR tag coordinates: ", target)
    print("Point in camera coordinates: ", point_cam)
    print("Rotation matrix rVec:        ", rvec)

    print("Rotation around x axis: ", (180 - angles[0]))
    print("Rotation around y axis: ", -angles[1])
    print("Rotation around z axis: ", -angles[2])

    return tvec, angles
