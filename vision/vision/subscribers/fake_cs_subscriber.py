import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            Int8, "ROVER/HD_element_id", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.data = 20

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")
        self.data = msg.data

    def get_data(self):
        return self.data


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


# if __name__ == "__main__":
#     main()
