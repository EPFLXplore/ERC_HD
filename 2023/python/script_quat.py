import cv2
import numpy as np
from numpy import sqrt
import cv2 as cv


# https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
def quaternion_from_rotation_matrix(rotation_matrix):

    # practical
    m = rotation_matrix

    # trace of the rotation matrix
    trace = np.sum(np.diag(rotation_matrix))

    if(trace > 0):

        S = sqrt(trace + 1.0) * 2 # S = 4 * qw
        check_S(S)

        qw = 0.25 * S
        qx = (m[2,1] - m[1,2]) / S
        qy = (m[0,2] - m[2,0]) / S
        qz = (m[1,0] - m[0,1]) / S

    # If the trace of the matrix is less than or equal to zero then identify which major diagonal element has the greatest value.

    elif m[0,0] > m[1,1] and m[0,0] > m[2,2]: # case m[0,0]

        S = sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2 # S = 4 * qx
        check_S(S)

        qw = (m[2,1] - m[1,2]) / S
        qx = 0.25 * S
        qy = (m[0,1] + m[1,0]) / S
        qz = (m[0,2] + m[2,0]) / S

    elif m[1,1] > m[2,2]: # case m[1,1]

        S = sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2 # S = 4 * qy
        check_S(S)

        qw = (m[0,2] - m[2,0]) / S
        qx = (m[0,1] + m[1,0]) / S
        qy = 0.25 * S
        qz = (m[1,2] + m[2,1]) / S

    else: # case m[2,2]

        S = sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2 # S = 4 * qz
        check_S(S)

        qw = (m[1,0] - m[0,1]) / S
        qx = (m[0,2] + m[2,0]) / S
        qy = (m[1,2] + m[2,1]) / S
        qz = 0.25 * S

    # return the quaternion info in [x,y,z,w] form
    return [qx, qy, qz, qw]

def quaternion_from_rVec(rVec):

    # convert rotation vector to rotation matrix
    rot_matrix, jacobian = cv.Rodrigues(rVec)

    return quaternion_from_rotation_matrix(rot_matrix)

### PRECONDITIONS

# make sure that S is never 0 to avoid division by 0
def check_S(S):
    assert (S != 0, f"S different than 0 expected, got: {S}")






