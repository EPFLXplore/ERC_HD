import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
import cv2  # OpenCV library
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import time
import os

from hd_interfaces.msg import CompressedRGBD  # Custom message type
import numpy as np


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_subscriber")
        self.get_logger().info("Image Subscriber Node Started")
        self.path = "./captured_images"
        os.makedirs(self.path, exist_ok=True)
        self.last_image_time = time.time()

        # Initialize FPS calculation variables
        self.prev_time = time.time()
        self.fps = 0

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            CompressedImage,
            "/hd/perception/image",
            self.listener_callback,
            10,
        )

        self.camera_sub = self.create_subscription(
            CompressedRGBD,
            "/HD/camera/rgbd",
            self.camera_callback,
            1,
        )

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def update_fps(self):
        """
        Calculate the FPS of the video stream.
        """
        current_time = time.time()
        time_diff = current_time - self.prev_time
        self.fps = 1.0 / time_diff
        self.prev_time = current_time

    def draw_fps(self, frame):
        """
        Draw the FPS on the frame.
        """
        cv2.putText(
            frame,
            f"FPS: {self.fps:4.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        self.update_fps()

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.compressed_imgmsg_to_cv2(data)

        self.draw_fps(current_frame)

        # Display image
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

    def camera_callback(self, rgbd_msg: CompressedRGBD):
        self.get_logger().info("Receiving camera RGBD frame")
        rgb = self.br.compressed_imgmsg_to_cv2(rgbd_msg.color)

        # Convert the byte data to a numpy array of uint16
        # depth_image = np.frombuffer(rgbd_msg.depth.data, dtype=np.uint16)
        depth_image = self.br.compressed_imgmsg_to_cv2(rgbd_msg.depth, "tiff")

        # Reshape the numpy array to match the image dimensions
        depth_image = depth_image.reshape((rgbd_msg.depth.height, rgbd_msg.depth.width))
        print(f"depth_image: {depth_image.shape}")
        print(f"rgbd type: {depth_image.dtype}")
        self.update_fps()
        self.draw_fps(rgb)
        cv2.imshow("camera", rgb)
        cv2.waitKey(1)

        # aruco_pose = self.pipeline_manager.process_rgb(rgb)
        # self.aruco_pub.publish(PoseMsg.create_message(*aruco_pose))


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
