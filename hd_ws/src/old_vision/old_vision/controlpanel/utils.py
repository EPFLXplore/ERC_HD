import numpy as np

def get_coords(top_left, height, width):
    # to get bottom buttons remove height from y because y axis is pointing up  
    top_right = [top_left[0] + width, top_left[1]]
    bottom_left = [top_left[0], top_left[1] - height]
    bottom_right = [top_left[0] + width, top_left[1] - height]
    return np.array([top_left, top_right, bottom_right, bottom_left ])