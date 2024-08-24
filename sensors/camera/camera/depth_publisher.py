import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class DepthImagePublisher(Node):
    def __init__(self):
        super().__init__("depth_image_publisher")
        self.publisher_ = self.create_publisher(Image, "depth_image", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create a dummy depth image for demonstration (480x640)
        depth_image = np.random.randint(0, 65535, (480, 640), dtype=np.uint16)

        # Convert the numpy array to bytes
        depth_image_bytes = depth_image.tobytes()

        # Create an Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_frame"
        msg.height = depth_image.shape[0]
        msg.width = depth_image.shape[1]
        msg.encoding = "mono16"  # Encoding for uint16 depth images
        msg.is_bigendian = False
        msg.step = msg.width * 2  # 2 bytes per pixel
        msg.data = depth_image_bytes

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing depth image")


def main(args=None):
    rclpy.init(args=args)
    depth_image_publisher = DepthImagePublisher()
    rclpy.spin(depth_image_publisher)

    # Shutdown the ROS 2 node
    depth_image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
