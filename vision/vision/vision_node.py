import os

print(f"cwd: {os.getcwd()}")
import sys

sys.path.append("./src/vision/vision")
# sys.path.append("./src/vision/vision")
# sys.path.append("./src/vision/vision/publishers")

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
import cv2 as cv
import numpy as np

from vision.controlpanel.control_panel import ControlPanel
from vision.stereo_camera import StereoCamera
from vision.controlpanel.control_panel import ControlPanel
from vision.utils import show, translation_rotation


# import os

# from image_publisher import ImagePublisher
# from target_pose_publisher import TargetPosePublisher
# from target_publisher import TargetPublisher

from vision.publishers.image_publisher import ImagePublisher
from vision.publishers.target_pose_publisher import TargetPosePublisher
from vision.publishers.target_publisher import TargetPublisher
from vision.publishers.depth_publisher import DepthPublisher

from vision.subscribers.fake_cs_subscriber import MinimalSubscriber
import threading


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
        self.depth_publisher = DepthPublisher()

        # Initialize subscribers
        self.subscriber = MinimalSubscriber()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.subscriber)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

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
        self.task = 20

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

            # projected_center = self.control_panel.panels["A"].project([0, 0, 0])
            # print(projected_center)

            point2project, rvec, tvec = self.control_panel.get_target()
            ar_translation, ar_quaternion = translation_rotation([0, 0, 0], rvec, tvec)

            translation, quaternion = translation_rotation(point2project, rvec, tvec)
            self.target_pose_publisher.publish(
                translation, quaternion, ar_translation, ar_quaternion, self.task
            )
            # print(translation, quaternion)

        # show(frame, depth)

        # key = cv.waitKey(1)
        # if key == 27:
        #     cv.destroyAllWindows()
        #     return
        # elif key in self.POSSIBLE_PANELS:
        #     self.control_panel.select_panel(key)
        self.task = self.subscriber.get_data()
        self.control_panel.set_target(self.task)
        self.image_publisher.publish(frame)
        self.depth_publisher.publish(depth)
        # self.image_publisher._logger.info(
        #     f"current task: {self.task:2d}, target panel {self.control_panel.get_selected_panel().name}"
        # )

        self.image_publisher._logger.info(
            f"current task: {self.task:3d}, target panel {self.control_panel.get_selected_panel().name}"
        )


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
