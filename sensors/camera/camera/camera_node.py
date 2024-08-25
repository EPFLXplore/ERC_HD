# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from custom_msg.srv import CameraParams
from custom_msg.msg import CompressedRGBD


from .interfaces.monocular_camera_interface import MonocularCameraInterface
from .camera_factory import CameraFactory
import time


class CameraNode(Node):
    """
    Create an CameraNode class, publishes video frames to the video_frames topic
    """

    def __init__(self, camera: MonocularCameraInterface):
        super().__init__("camera_node")

        self.camera_intrinsics_srv = self.create_service(
            CameraParams,
            "/HD/camera/params",
            self.camera_params_callback,
        )

        self.camera = camera

        # to the HD/vision/video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(CompressedRGBD, "HD/camera/rgbd", 1)
        self.get_logger().info("Image Publisher Created")

        # We will publish a message every 0.1 seconds
        timer_period = 0.035  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.rgbd_callback)

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    # def timer_callback(self):

    def rgbd_callback(self):
        """
        Callback function.
        Publishes a frame to the video_frames topic
        """
        color, depth = self.camera.get_rgbd()

        msg = CompressedRGBD()

        depth_msg = self.bridge.cv2_to_imgmsg(depth, "mono16")

        msg.depth = depth_msg

        msg.color = self.bridge.cv2_to_compressed_imgmsg(color)

        self.publisher_.publish(msg)

    def camera_params_callback(self, request, response):
        intrinsics = self.camera.get_intrinsics()
        distortion_coefficients = self.camera.get_coeffs()
        response.fx = intrinsics["fx"]
        response.fy = intrinsics["fy"]
        response.cx = intrinsics["cx"]
        response.cy = intrinsics["cy"]
        response.distortion_coefficients = distortion_coefficients
        self.get_logger().info(
            f"Provided intrinsics: fx={response.fx}, fy={response.fy}, cx={response.cx}, cy={response.cy}"
        )
        self.get_logger().info(
            f"Provided distortion coefficients: {response.distortion_coefficients}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("camera_selector")
    camera_type = node.declare_parameter("camera_type").value

    camera = CameraFactory.create_camera(camera_type)

    camera_node = CameraNode(camera=camera)
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


"""
README

Test:
- convert rbdf message to 2 compressed images and try tiff
- 

"""
