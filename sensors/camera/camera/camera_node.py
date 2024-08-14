# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from hd_interfaces.srv import GetCameraIntrinsics, GetCameraDistortionCoefficients


import sys

from camera.realsense_camera import RealSenseCamera
import os

print(f"cwd: {os.getcwd()}")

sys.path.append("./src/vision/vision")


class CameraNode(Node):
    """
    Create an CameraNode class, publishes video frames to the video_frames topic
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("image_publisher")
        self.camera_intrinsics_srv = self.create_service(
            GetCameraIntrinsics,
            "/HD/camera/get_camera_intrinsics",
            self.get_intrinsics_callback,
        )

        self.camera_distortion_coefficients_srv = self.create_service(
            GetCameraDistortionCoefficients,
            "/HD/camera/distortion_coefficients",
            self.get_distortion_coefficients_callback,
        )

        self.camera = RealSenseCamera()

        # to the HD/vision/video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(CompressedImage, "HD/camera/rgb", 1)
        self.get_logger().info("Image Publisher Created")

        # We will publish a message every 0.1 seconds
        timer_period = 0.035  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # def timer_callback(self):

    def timer_callback(self):
        """
        Callback function.
        """
        depth = self.camera.get_depth()
        frame = self.camera.get_image()
        """
        Publishes a frame to the video_frames topic
        """
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")

    def get_intrinsics_callback(self, request, response):
        intrinsics = self.camera.get_intrinsics()
        response.fx = intrinsics["fx"]
        response.fy = intrinsics["fy"]
        response.cx = intrinsics["cx"]
        response.cy = intrinsics["cy"]
        self.get_logger().info(
            f"Provided intrinsics: fx={response.fx}, fy={response.fy}, cx={response.cx}, cy={response.cy}"
        )
        return response

    def get_distortion_coefficients_callback(self, request, response):
        distortion_coefficients = self.camera.get_coeffs()
        response.distortion_coefficients = distortion_coefficients
        self.get_logger().info(
            f"Provided distortion coefficients: {response.distortion_coefficients}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = CameraNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
