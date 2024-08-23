import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type

from hd_interfaces.msg import CompressedRGBD  # Custom message type

from .module_manager import ModuleManager

import numpy as np
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import Pose

from .pipeline_manager import PipelineManager
from .handlers.pose_msg import PoseMsg


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        # Initialize Module Manager
        # self.pipeline_manager = PipelineManager()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            CompressedImage, "HD/camera/rgb", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            CompressedImage, "HD/camera/depth", self.depth_callback, 10
        )

        self.rgbd_sub = self.create_subscription(
            CompressedRGBD, "/HD/camera/rgbd", self.rgbd_callback, 1
        )

        self.get_logger().info(
            "Perception Node started with rock detection and size estimation."
        )

        self.processed_rgb_pub = self.create_publisher(
            CompressedImage, "/hd/perception/image", 1
        )

        self.aruco_pub = self.create_publisher(Pose, "/hd/perception/aruco", 10)

        self.bridge = CvBridge()

    def rgbd_callback(self, rgbd_msg: CompressedRGBD):
        rgb = self.bridge.compressed_imgmsg_to_cv2(rgbd_msg.color)
        depth_image = self.bridge.imgmsg_to_cv2(rgbd_msg.depth, "mono16")

        # aruco_pose = self.pipeline_manager.process_rgb(rgb)
        # self.aruco_pub.publish(PoseMsg.create_message(*aruco_pose))
        rgb_msg = self.bridge.cv2_to_compressed_imgmsg(rgb)
        self.processed_rgb_pub.publish(rgb_msg)

    def rgb_callback(self, msg):
        rgb_frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.pipeline_manager.process_rgb(rgb_frame)

    def depth_callback(self, msg):
        depth_frame = self.bridge.imgmsg_to_cv2(msg)
        self.pipeline_manager.process_depth(depth_frame)

        # Perform post-processing
        # self.module_manager.post_process(rgb_frame, depth_frame)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
