import sys

sys.path.append("./src/vision/vision")

import rclpy
from image_publisher import ImagePublisher
from target_pose_publisher import TargetPosePublisher
from target_publisher import TargetPublisher

from rclpy.executors import SingleThreadedExecutor


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create publisher nodes
    image_publisher = ImagePublisher()
    pose_target_publisher = TargetPosePublisher()
    target_publisher = TargetPublisher()

    executor = SingleThreadedExecutor()
    executor.add_node(pose_target_publisher)
    executor.add_node(target_publisher)
    executor.add_node(image_publisher)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        image_publisher.destroy_node()
        target_publisher.destroy_node()
        target_publisher.destroy_node()


if __name__ == "__main__":
    main()
