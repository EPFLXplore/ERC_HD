import cv2 as cv
import numpy as np

def compute_projection_matrix(intrinsics, rVec, tVec):

    # convert rotation vector to rotation matrix
    rot_matrix, jacobian = cv.Rodrigues(rVec)

    # put it into right shape for hstack 
    tVec = [[tVec[0,0]], [tVec[0,1]], [tVec[0,2]]]

    # concatenate rotation matrix with translation vector to get the extrinsics matrix
    extrinsics = np.hstack((rot_matrix, tVec))

    # multiply with intrinsics(camera_matrix) to get the projection matrix
    projection_matrix = intrinsics @ extrinsics
    
    return projection_matrix

# compute homography matrix
def get_homography_matrix(perspective_matrix: np.array)-> np.array:

    # Take 5 homogenous points on the floor(Unit is in cm) (x,y,z,1) we need at least 4
    pts_dst = np.array([[0, 0, 0, 1],
                        [0, 1, 0, 1],
                        [1, 0, 0, 1],
                        [1, 1, 0, 1],
                        [0, 0, 0, 1]
                        ])

    # Obtain respective homogenous points on the image plane
    pts_src = (perspective_matrix @ pts_dst.T).T

    # convert homogenous coordinates to cartesian coorndinates
    pts_src_cart = np.array([[x / w, y / w] for x, y, w in pts_src])
    pts_dst_cart = np.array([[x / w, y / w] for x, y, z, w in pts_dst])

    # find the 3x3 Homography Matrix for transforming image plane to floor plane
    H, status = cv.findHomography(pts_src_cart, pts_dst_cart)

    return H

