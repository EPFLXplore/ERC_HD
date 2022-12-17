import numpy as np
import cv2 as cv

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

    print(R.shape)
    print(M.shape)
    print(point.shape)

    x, y, z, n = R @ M @ point.T

    return [x/n, y/n, z/n]

