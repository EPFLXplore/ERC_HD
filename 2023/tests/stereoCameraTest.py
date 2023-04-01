import sys 
import os
sys.path.append('/'.join(os.getcwd().split('/')[:-1]))

import cv2 as cv
import numpy as np

from python.stereoCamera import StereoCamera


MAX_DIST = 25500 # everything past 2.55 meters is set to 2.55 meters 


camera = StereoCamera()
while True:
    depth = camera.get_depth()
    frame = camera.get_image()

    depth[ depth > MAX_DIST] = MAX_DIST
    depth = 255.0 * depth / depth.max()

    depth = depth.astype(np.uint8)
    depth_3_channel = cv.cvtColor(depth, cv.COLOR_GRAY2BGR)

    numpy_horizontal = np.hstack((frame, depth_3_channel))

    cv.imshow('RealSense', numpy_horizontal) 

    key = cv.waitKey(1)
    if key == 27:
        cv.destroyAllWindows()
        break

print("coefficients")
print(camera.get_coeffs())

print("intrinsics")
print(camera.get_intrinsics())
 
   
    

