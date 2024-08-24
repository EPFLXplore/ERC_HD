import numpy as np


"""
    position 0 --- 1
             |     |
             3 --- 2
"""


def center_hw_to_corners(center, height: int, width: int):
    # to get bottom buttons remove height from y because y axis is pointing up
    top_left = np.array([center[0] - width // 2, center[1] + height // 2])
    top_right = np.array([center[0] + width // 2, center[1] + height // 2])
    bottom_left = np.array([center[0] - width // 2, center[1] - height // 2])
    bottom_right = np.array([center[0] + width // 2, center[1] - height // 2])
    return np.array([top_left, top_right, bottom_right, bottom_left])


def corners_to_center_hw(corners: np.ndarray):
    center = np.array([corners[:, 0].mean(), corners[:, 1].mean()])
    width = int(np.linalg.norm(corners[0] - corners[1]))
    height = int(np.linalg.norm(corners[0] - corners[3]))
    return center, height, width


def top_left_hw_to_corners(top_left: np.ndarray, height: int, width: int):
    # to get bottom buttons remove height from y because y axis is pointing up
    top_right = np.array([top_left[0] + width, top_left[1]])
    bottom_left = np.array([top_left[0], top_left[1] - height])
    bottom_right = np.array([top_left[0] + width, top_left[1] - height])
    return np.array([top_left, top_right, bottom_right, bottom_left])


def corners_to_top_left_hw(corners: np.ndarray):
    top_left = corners[0]
    width = int(np.linalg.norm(corners[0] - corners[1]))
    height = int(np.linalg.norm(corners[0] - corners[3]))
    return top_left, height, width
