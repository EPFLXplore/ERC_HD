from vision.stereo_camera import StereoCamera
import cv2 as cv
import numpy as np
from vision.controlpanel.control_panel import ControlPanel
from vision.utils import show, translation_rotation
from vision.vision_publisher import VisionPublisher
from vision.camera_flux import CameraFluxPublisher
from vision.ar_tag_publisher import ARTagsPublisher
from vision.toggle_cameras import HDToggleCamerasSubscriber
from vision.element_id import ElementIdSubscriber

import rclpy
from rclpy.node import Node
import threading

# =========================================================================================================
def main(args=None):
    rclpy.init()

    camera = StereoCamera()

    BUTTON_VALUES = [ x for x in range( 10)]
    control_panel = ControlPanel(camera.get_intrinsics() , camera.get_coeffs(), BUTTON_VALUES)

    MAX_DIST = 25500 # everything past 2.55 meters is set to 2.55 meters 
    POSSIBLE_PANELS = set([ord('1'), ord('2'), ord('a')])

    vision_node = Node("HD_vision_node")

    ############
    # PUBLISHERS
    ############
    publisher = VisionPublisher(vision_node)
    # camera_publisher = CameraFluxPublisher(vision_node)
    tag_publisher = ARTagsPublisher(vision_node)

    ############
    # SUBSCRIBERS
    ############
    # toggle_cameras_subscriber = HDToggleCamerasSubscriber(vision_node, camera_publisher)
    element_id_subscriber = ElementIdSubscriber(vision_node)

    threading.Thread(target=rclpy.spin, args=(vision_node,), daemon=True).start()

    rate = vision_node.create_rate(10)  # 10hz


    while True:
        depth = camera.get_depth()
        frame = camera.get_image()

        if control_panel.detect_ar_tag(frame):
            control_panel.project()
            control_panel.draw(frame)

            point2project, rvec, tvec = control_panel.get_target()
            translation, quaternion = translation_rotation(point2project, rvec, tvec)

            msg = publisher.create_panelobject_message(0 ,*translation,*quaternion) # TODO change 0 to what it is         
            publisher.publish_inform(msg)
            rate.sleep()

        # show(frame, depth)


        key = cv.waitKey(1)
        if key == 27:
            cv.destroyAllWindows()
            break
        elif key in POSSIBLE_PANELS:
            control_panel.select_panel(key) 
            control_panel.set_target()
    




if __name__ == '__main__':
    main()