import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class VisionPublisher(Node):

    def __init__(self):
        super().__init__('vision_distance_publisher')
        self.publisher_=self.create_publisher(String, 'distance_topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.projector = projector
        self.i = 0

    
    def timer_callback(self):
        msg = String()
        msg.data = 'Distance from Panel: %d ' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)


    vision_publisher = VisionPublisher()

    rclpy.spin(vision_publisher)

if __name__ == '__main__':
    main()
