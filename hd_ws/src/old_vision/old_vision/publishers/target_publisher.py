from rclpy.node import Node
from std_msgs.msg import Int32


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")

        self.publisher_ = self.create_publisher(Int32, "target", 10)
        self.get_logger().info("Target Publisher Created")

        self.target = 0

    # def timer_callback(self):
    #     msg = Int32()
    #     msg.data = self.target
    #     self.publisher_.publish(msg)
    #     self.target += 1

    def publish(self, target):
        msg = Int32()
        msg.data = target
        self.publisher_.publish(msg)
