import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class VisionSubscriber(Node):

    def __init__(self):
        super().__init__('vision_distance_subscriber')
        self.subscription = self.create_subscription(
            String,
            'distance_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = VisionSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()
