import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type

from hd_interfaces.msg import CompressedRGBD  # Custom message type

from .module_manager import ModuleManager

import numpy as np
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import Pose


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        # Initialize Module Manager
        self.module_manager = ModuleManager()

        # # Add modules
        # rock_detection = RockDetectionModule(self)
        # rock_size_estimation = RockSizeEstimationModule(self)

        # self.module_manager.add_module(rock_detection)
        # self.module_manager.add_module(rock_size_estimation)
        # Add more modules as needed

        # Subscribers
        self.rgb_sub = self.create_subscription(
            CompressedImage, "HD/camera/rgb", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            CompressedImage, "HD/camera/depth", self.depth_callback, 10
        )

        self.rgbd_sub = self.create_subscription(
            CompressedRGBD, "/HD/camera/rgbd", self.rgbd_callback, 10
        )

        self.get_logger().info(
            "Perception Node started with rock detection and size estimation."
        )

        self.perception_pub = self.create_publisher(
            CompressedImage, "/hd/perception/image", 10
        )

        self.aruco_pub = self.create_publisher(Pose, "/hd/perception/aruco", 10)

        self.bridge = CvBridge()

    def rgbd_callback(self, rgbd_msg: CompressedRGBD):
        rgb = self.bridge.compressed_imgmsg_to_cv2(rgbd_msg.color)

        # Convert the byte data to a numpy array of uint16
        depth_image = np.frombuffer(rgbd_msg.depth.data, dtype=np.uint16)

        # Reshape the numpy array to match the image dimensions
        depth_image = depth_image.reshape((rgbd_msg.depth.height, rgbd_msg.depth.width))
        print(f"depth_image: {depth_image.shape}")
        print(f"rgbd type: {depth_image.dtype}")

        aruco_pose = self.module_manager.process_rgb(rgb)
        if aruco_pose:
            pose_msg = Pose()
            pose_msg.x = aruco_pose[0]
            pose_msg.position.y = aruco_pose[1]
            pose_msg.position.z = aruco_pose[2]
            self.aruco_pub.publish(pose_msg)

    def rgb_callback(self, msg):
        rgb_frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.module_manager.process_rgb(rgb_frame)

    def depth_callback(self, msg):
        depth_frame = self.bridge.imgmsg_to_cv2(msg)
        self.module_manager.process_depth(depth_frame)

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
