import os

print(f"cwd: {os.getcwd()}")
import sys

sys.path.append("./src/old_vision/old_vision")
# sys.path.append("./src/old_vision/old_vision")
# sys.path.append("./src/old_vision/old_vision/publishers")

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
import cv2 as cv
import numpy as np

from old_vision.stereo_camera import StereoCamera
from old_vision.controlpanel.control_panel import ControlPanel
from old_vision.utils import show, translation_rotation

# import os

# from image_publisher import ImagePublisher
# from target_pose_publisher import TargetPosePublisher
# from target_publisher import TargetPublisher

from old_vision.publishers.image_publisher import ImagePublisher
from old_vision.publishers.target_pose_publisher import TargetPosePublisher
from old_vision.publishers.target_publisher import TargetPublisher


class VisionNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("aaa_image_publisher")

        # Initialize publishers
        self.image_publisher = ImagePublisher()
        self.target_pose_publisher = TargetPosePublisher()
        self.target_publisher = TargetPublisher()

        # We will publish a message every 0.1 seconds
        timer_period = 0.04  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.camera = StereoCamera()

        BUTTON_VALUES = [x for x in range(10)]
        self.control_panel = ControlPanel(
            self.camera.get_intrinsics(), self.camera.get_coeffs(), BUTTON_VALUES
        )

        self.MAX_DIST = 25500  # everything past 2.55 meters is set to 2.55 meters
        self.POSSIBLE_PANELS = set([ord("1"), ord("2"), ord("a")])

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        depth = self.camera.get_depth()
        frame = self.camera.get_image()

        if self.control_panel.detect_ar_tag(frame):
            self.control_panel.project()
            self.control_panel.draw(frame)

            point2project, rvec, tvec = self.control_panel.get_target()
            translation, quaternion = translation_rotation(point2project, rvec, tvec)
            self.target_pose_publisher.publish(translation, quaternion)
            # print(translation, quaternion)

        # show(frame, depth)

        # key = cv.waitKey(1)
        # if key == 27:
        #     cv.destroyAllWindows()
        #     return
        # elif key in self.POSSIBLE_PANELS:
        #     self.control_panel.select_panel(key)
        #     self.control_panel.set_target()

        self.image_publisher.publish(frame)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = VisionNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
