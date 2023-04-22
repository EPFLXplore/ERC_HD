from python.stereoCamera import StereoCamera
import cv2 as cv
import numpy as np
from controlpanel.control_panel import ControlPanel


def show(frame, depth):
    depth[ depth > MAX_DIST] = MAX_DIST
    depth = 255.0 * depth / depth.max()

    depth = depth.astype(np.uint8)
    depth_3_channel = cv.cvtColor(depth, cv.COLOR_GRAY2BGR)

    numpy_horizontal = np.hstack((frame, depth_3_channel))

    cv.imshow('RealSense', numpy_horizontal) 

camera = StereoCamera()

BUTTON_VALUES = [ x for x in range( 10)]
control_panel = ControlPanel(camera.get_intrinsics() , camera.get_coeffs(), BUTTON_VALUES)

MAX_DIST = 25500 # everything past 2.55 meters is set to 2.55 meters 
POSSIBLE_PANELS = set([ord('1'), ord('2'), ord('a')])


while True:
    depth = camera.get_depth()
    frame = camera.get_image()

    
    cv.putText(frame, control_panel.selected_panel, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)

    control_panel.draw(frame)
    test = np.array([[10,10],[10,200], [200,200], [200, 10]])
    print(test.shape)
    cv.polylines(frame, [test], True, (0, 0, 255), 2)
    show(frame, depth)

    key = cv.waitKey(1)


    if key == 27:
        cv.destroyAllWindows()
        break
    elif key in POSSIBLE_PANELS:
        control_panel.select_panel(key)
   