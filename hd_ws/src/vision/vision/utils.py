import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R
from numpy import linalg


def show(frame, depth, max_dist = 25500): # everything past 2.55 meters is set to 2.55 meters 
    depth[ depth > max_dist] = max_dist
    depth = 255.0 * depth / depth.max()

    depth = depth.astype(np.uint8)
    depth_3_channel = cv.cvtColor(depth, cv.COLOR_GRAY2BGR)

    numpy_horizontal = np.hstack((frame, depth_3_channel))

    cv.imshow('RealSense', numpy_horizontal) 


def rvec2quat(rVec):
    # conversion to rotation matrix
    r = R.from_rotvec(rVec)
    try:
        rot_mat = r.as_dcm()
    except:
        rot_mat = r.as_matrix()

    # reshape to 3*3
    rot_mat = rot_mat.T.reshape((3,3))

    # change of reference
    rot_mat = linalg.inv(rot_mat)

    # convert to quat
    try:
        r2 = R.from_dcm(rot_mat)
    except:
        r2 = R.from_matrix(rot_mat)
    quat = r2.as_quat()
    
    return quat


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
    R, _jacobian = cv.Rodrigues(rVec)
    
    # change of perspective
    tVec = -linalg.inv(R) @ tVec.T
    tVec = tVec.reshape(-1)
    
    # append extra row and column
    R = np.vstack((R, [0, 0, 0]))
    R = np.hstack((R, [[0], [0], [0], [1]]))

    # identity 4x4
    M = np.eye(4)
    M[0:3, 3] = -tVec

    x, y, z, n = R @ M @ point
    

    return np.array([x, y, z]) / n

def translation_rotation(point, rVec, tVec):
    translation = camera_projection(point, rVec, tVec)
    quaternion = rvec2quat(rVec)
    return translation, quaternion