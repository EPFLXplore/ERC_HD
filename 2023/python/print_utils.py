"""
File containing methods to print different information in a convenient manner
"""
import numpy as np
from numpy import pi


def print_rvec(rVec, in_degrees=False, decimals=0, on_same_line=False, returns_string=False):
    """
    prints a rotation vector (rVec) in a user friendly readable manner 
    rVec should be in radians in the range [-pi,pi)
    """
    if in_degrees:
        rVec *= 180 / pi
    
    rVec = rVec.round(decimals)
    if decimals == 0:
        rVec = rVec.astype(int)

    

    string = "Rotation vector: "

    for coord in rVec:
        string +=  f"{coord:5}, "

    if returns_string:
        return string
    else:
        end = '\n'
        if on_same_line:
            end = '\r'
        print(string, end=end)


def print_tvec(tVec , decimals=0, on_same_line=False, returns_string=False):
    # print("Point in camera coordinates: ", point_cam.round(2), end='\r')
    
    tVec *= 10 # converts cm to mm

    tVec = tVec.round(decimals)
    if decimals == 0:
        tVec = tVec.astype(int)

    string = " Translation vector: "
    for coord in tVec:
        string +=  f"{coord:5}, "
        
    if returns_string:
        return string
    else:
        end = '\n'
        if on_same_line:
            end = '\r'
        print(string, end=end)
    