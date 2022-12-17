import cv2 as cv
import numpy as np

def inverse_project_P(point, projection_matrix):
    """"
    point: (u, v) pixel coordinates
    projection_matrix: 3x4 matrix ( intrinsics @ [R|t] )
    """

    # append 1 to pixel coordinates for homogenous coordinates to satisfy the equation
    point.append(1)

    # since we are projecting onto a plan (z=0) the 3rd column of the projection matrix can be ignored
    # get partial projection matrix (3x4) -> (3x3)
    partial_pm = projection_matrix[:, [0,1,3]]

    # now it is invertible
    # invert partial projection matrix
    inv_partial_pm = np.linalg.inv(partial_pm)

    # compute the corresponding (x,y)
    world_coord = inv_partial_pm @ np.array(point).T

    # need to normalize !!!!
    return world_coord[:2]/world_coord[2]



#### if the above code doesn't work implement the inverse projection with homography matrix (for an arbitrary value of z)

# a method to compute the inverse of projection matrix by homography
def inverse_project_H(image_coordinates, H):

    """"
    image_coordinates: 2d pixel coordinates (x,y)
    H: 3x3 Homography matrix np.array[3x3]
    """

    # adding 1 for homogenous coordinate system
    image_coordinates.append(1)

    # world coordinates 
    x, y, w = H @ np.array(image_coordinates).T
    
    # don't forget to normalize
    return [x / w, y / w]
