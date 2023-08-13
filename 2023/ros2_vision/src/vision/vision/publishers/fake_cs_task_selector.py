import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32


class FakeCSTaskSelector(Node):
    panel_a_task_numbers = [20 + i for i in range(10)]
    panel_b1_task_numbers = [0, 10]
    panel_b2_task_numbers = [30]

    def __init__(self):
        super().__init__("fake_cs_task_selector")

        self.accepted_tasks = set(
            self.panel_a_task_numbers
            + self.panel_b1_task_numbers
            + self.panel_b2_task_numbers
        )

        self.publisher_ = self.create_publisher(Int32, "cs_task", 10)
        self.get_logger().info("Fake CS Task Selector Created")

        self.cs_task = 0
        timer_period = 0.5

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def select_task(self):
        task = -1
        while task not in self.accepted_tasks:
            task = input("Enter task number: ")

        self.cs_task = task


def main(args=None):
    rclpy.init(args=args)

    fake_cs_task_selector = FakeCSTaskSelector()

    while rclpy.ok():
        fake_cs_task_selector.select_task()
        fake_cs_task_selector.publisher_.publish(Int32(data=10))

    fake_cs_task_selector.destroy_node()
    rclpy.shutdown()
