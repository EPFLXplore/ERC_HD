import sys

sys.path.append("./src/vision/vision")

import rclpy
from image_publisher import ImagePublisher
from target_pose_publisher import TargetPosePublisher
from target_publisher import TargetPublisher


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create publisher nodes
    image_publisher = ImagePublisher()
    pose_target_publisher = TargetPosePublisher()
    target_publisher = TargetPublisher()

    # spin both nodes
    rclpy.spin(pose_target_publisher)
    rclpy.spin(target_publisher)


# def main(args=None):
#     # Initialize the rclpy library
#     rclpy.init(args=args)

#     # Create the node
#     image_publisher = ImagePublisher()

#     # Spin the node so the callback function is called.
#     rclpy.spin(image_publisher)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     image_publisher.destroy_node()

#     # Shutdown the ROS client library for Python
#     rclpy.shutdown()


if __name__ == "__main__":
    main()
