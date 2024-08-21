# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage, Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from hd_interfaces.srv import GetCameraIntrinsics, GetCameraDistortionCoefficients
from hd_interfaces.msg import CompressedRGBD


from .interfaces.monocular_camera_interface import MonocularCameraInterface
from .camera_factory import CameraFactory


class CameraNode(Node):
    """
    Create an CameraNode class, publishes video frames to the video_frames topic
    """

    def __init__(self, camera: MonocularCameraInterface):
        super().__init__("camera_node")

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

        self.camera = camera

        # to the HD/vision/video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(CompressedRGBD, "HD/camera/rgbd", 1)
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
        msg = CompressedRGBD()

        # Convert the numpy array to bytes
        depth_image_bytes = depth.tobytes()

        # Create an Image message
        depth_msg = Image()
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "camera_depth_frame"
        depth_msg.height = depth.shape[0]
        depth_msg.width = depth.shape[1]
        depth_msg.encoding = "mono16"  # Encoding for uint16 depth images
        depth_msg.is_bigendian = False
        depth_msg.step = depth_msg.width * 2  # 2 bytes per pixel
        depth_msg.data = depth_image_bytes

        msg.depth = depth_msg
        msg.color = self.bridge.cv2_to_compressed_imgmsg(frame)

        self.publisher_.publish(msg)

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

    node = rclpy.create_node("camera_selector")
    camera_type = node.declare_parameter("camera_type", "monocular").value

    camera = CameraFactory.create_camera(camera_type)

    camera_node = CameraNode(camera=camera)
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
