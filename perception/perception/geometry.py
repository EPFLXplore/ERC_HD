import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from numpy import linalg
from numpy import ndarray


def rvec2quat(rVec):
    # conversion to rotation matrix
    r = R.from_rotvec(rVec)
    try:
        rot_mat = r.as_dcm()
    except:
        rot_mat = r.as_matrix()

    # reshape to 3*3
    rot_mat = rot_mat.T.reshape((3, 3))

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
    """ ""

    # append 1 for homogenous coordinates
    point = np.append(point, [1])

    # compute the rotation matrix
    R, _jacobian = cv2.Rodrigues(rVec)

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


# ===================================================================================


def point_2_homogeneous(point):
    return np.append(point, [1])


def homogeneous_2_point(homogeneous_point):
    return homogeneous_point[:-1]


def get_rotation_matrix(rvec):
    return cv2.Rodrigues(rvec)


def get_tranformation_matrix(rvec, tvec):
    R, what = cv2.Rodrigues(rvec)
    M = np.eye(4)
    M[:3, :3] = R
    if tvec.shape != (3,):
        # print(f"wrong shape for< tvec, {tvec.shape}")
        tvec = tvec.reshape(3)

    M[:3, 3] = tvec
    return M


def transform_and_project(point3d, M, intrinsics):
    return project_point(tf(M, point3d), intrinsics)


def project_point(point3d, intrinsics):
    homogeneous_projected_point = intrinsics @ point3d
    homogeneous_projected_point /= homogeneous_projected_point[2]
    # print(f"homogeneous_projected_point: {homogeneous_projected_point}")
    return homogeneous_2_point(homogeneous_projected_point)


def invert_homogeneous_matrix(M):
    """
    source: https://answers.opencv.org/question/73149/world-co-ordinates-and-object-co-ordinates/#73173

    Args:
        M (_type_): _description_
    """
    R = M[0:3, 0:3]
    t = M[0:3, 3]
    R_transpose = R.T
    inv_R = R_transpose
    inv_t = -R_transpose @ t
    inverted_M = np.eye(4)
    inverted_M[0:3, 0:3] = inv_R
    inverted_M[0:3, 3] = inv_t
    return inverted_M


def homo_2_vecs(M):
    assert M.shape == (4, 4)
    tvec = M[:3, 3]
    rvec, _ = cv2.Rodrigues(M[:3, :3])
    # rvec = rvec.reshape(3)
    print(M[:3, :3].shape)
    print(rvec)
    assert tvec.shape == (3,), f"tvec predicted: (3,), actual: {tvec.shape}"
    assert rvec.shape == (3, 1), f"rvec predicted: (3,1), actual: {rvec.shape}"
    return tvec, rvec


def tf(W: ndarray, point: ndarray):
    """_summary_

    Args:
        W (ndarray): Homogeneous transformation matrix
        point (ndarray): 3d point
    """
    return homogeneous_2_point(W @ point_2_homogeneous(point))


def tf_project(W, point, intrinsics):
    return project_point(tf(W, point), intrinsics)
