import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from custom_msg.srv import InitializeModel, Detect
import numpy as np
import cv2

class ModelClient(Node):
    def __init__(self):
        super().__init__('model_client')

        # Create a client for the InitializeModel service
        self.init_client = self.create_client(InitializeModel, '/HD/model_server/init_model')

        # Create a client for the Detect service
        self.detect_client = self.create_client(Detect, '/HD/model_server/detect')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Wait for the services to be available
        self.get_logger().info('Waiting for InitializeModel and Detect services...')
        self.init_client.wait_for_service()
        self.detect_client.wait_for_service()
        self.get_logger().info('Services are available.')

        # Call the initialize model service
        # self.call_initialize_model_service('src/models/brick_n_200.pt')

        # Generate a random image
        image = self.generate_random_image(480, 640)  # Example size: 480x640

        # Call the detect service
        self._logger.info('segment this image')
        self.call_detect_service(image)

    def call_initialize_model_service(self, model_path):
        request = InitializeModel.Request()
        request.model_path = model_path

        future = self.init_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Model initialized successfully.')
        else:
            self.get_logger().error('Failed to initialize model.')

    def call_detect_service(self, image):
        # Convert OpenCV image to CompressedImage
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(image)

        # Create a request for the Detect service
        request = Detect.Request()
        request.image = compressed_image_msg

        future = self.detect_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is not None: 
            if response.success:
                self.get_logger().info('Detection completed successfully.')
                # Process the received detection results if necessary
                result = future.result().image
                self.process_results(result)
        else:
            self.get_logger().error('Detection failed.')

    def generate_random_image(self, height, width):
        # Create a random image with the specified height and width
        random_image = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)
        return random_image

    def process_results(self, results):
        # Implement the logic to process the detection results here
        self.get_logger().info('Processing results...')
        # Example: Print the results
        print(results)


def main(args=None):
    rclpy.init(args=args)
    node = ModelClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
