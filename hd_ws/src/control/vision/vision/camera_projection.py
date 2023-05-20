import numpy as np
import cv2 as cv
import numpy.linalg as linalg


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
    R = np.array(R)

    # change of perspective
    tVec_np = np.array(tVec)
    tVec_np = -linalg.inv(R) @ tVec_np.T

    # append extra row and column
    R = np.vstack((R, [0, 0, 0]))
    R = np.hstack((R, [[0], [0], [0], [1]]))

    # identity 4x4
    id_3 = np.eye(3)

    id3 = np.vstack((id_3, [0, 0, 0]))

    M = np.hstack((id3, [[-tVec_np[0, 0]], [-tVec_np[1, 0]], [-tVec_np[2, 0]], [1]]))

    x, y, z, n = R @ M @ point.T

    return np.array([x, y, z]) / n


