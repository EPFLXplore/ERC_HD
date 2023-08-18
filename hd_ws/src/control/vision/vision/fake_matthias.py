import rclpy
from rclpy.node import Node

from interfaces.msg import PanelObject

from std_msgs.msg import String

class VisionSubscriber(Node):

    def __init__(self):
        super().__init__('vision_distance_subscriber')
        self.subscription = self.create_subscription(
            PanelObject,
            '/HD/vision/distance_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print('got message')
        
        self.get_logger().info('I heard: "%d"' % msg.id)
        self.get_logger().info('Orientation x: "%d"' % msg.pose.orientation.x)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = VisionSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()
