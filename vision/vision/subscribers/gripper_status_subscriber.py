# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
import cv2  # OpenCV library
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import time 
import os
from .distance_aruco import *
from std_msgs.msg import Float32

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
        self.path = './captured_images'
        os.makedirs(self.path, exist_ok=True)
        self.last_image_time = time.time()

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            CompressedImage, "HD/vision/video_frames", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # Create a publisher. This publisher will publish Float32 messages
        # to the "vision/distance" topic.
        self.publisher = self.create_publisher(Float32, "HD/vision/distance", 10)


        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()




    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.compressed_imgmsg_to_cv2(data)

        # detect aruco tags and draw distance on frame 
        distance,processed_img = dist_detection(current_frame, aruco_dict, parameters)
        print("type of distance=",type(distance))

        # Publish the distance
        self.publisher.publish(Float32(data=distance))

        self.get_logger().info('Publishing: "%s"' % distance)
        # Logging image like a boss
        # current_time = time.time()
        # if current_time - self.last_image_time > 1:
        #     cv2.imwrite(os.path.join(self.path, str(time.time()).replace('.', '') + '.png'), current_frame)
        #     self.last_image_time = current_time

        # Display image
        cv2.imshow('ArUco Marker Detection', processed_img)

        cv2.waitKey(1)


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





