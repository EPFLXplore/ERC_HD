from stereo_camera import StereoCamera
import cv2 as cv
import numpy as np
from controlpanel.control_panel import ControlPanel
from utils import show



camera = StereoCamera()

BUTTON_VALUES = [ x for x in range( 10)]
control_panel = ControlPanel(camera.get_intrinsics() , camera.get_coeffs(), BUTTON_VALUES)

MAX_DIST = 25500 # everything past 2.55 meters is set to 2.55 meters 
POSSIBLE_PANELS = set([ord('1'), ord('2'), ord('a')])


while True:
    depth = camera.get_depth()
    frame = camera.get_image()

    # TODO send to cs
    
    cv.putText(frame, control_panel.selected_panel, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)

    if control_panel.detect_ar_tag(frame):
        control_panel.project()
        control_panel.draw(frame)


    # control_panel.draw(frame)
    show(frame, depth)

    # select panel                                 OK
    # select target                                OK
    # compute necessary rotation and translation
    # draw target on image
    # control_panel.update(frame)

    key = cv.waitKey(1)
    if key == 27:
        cv.destroyAllWindows()
        break
    elif key in POSSIBLE_PANELS:
        control_panel.select_panel(key) 
        control_panel.set_target()
   