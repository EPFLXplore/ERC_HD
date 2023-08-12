from rclpy.node import Node


class TargetPublisher(Node):
    def __init__(self):
        super().__init__("target_publisher")

        self.publisher_ = self.create_publisher(int, "target", 10)
        self.get_logger().info("Target Publisher Created")

        timer_period = 0.5

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target = 0

    def timer_callback(self):
        self.publisher_.publish(self.target)
        self.target += 1
