import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8


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

        self.publisher_ = self.create_publisher(Int8, "ROVER/HD_element_id", 10)
        self.get_logger().info("Fake CS Task Selector Created")

        self.cs_task = 0

    def select_task(self):
        task = -1
        while task not in self.accepted_tasks:
            task = input("Enter task number: ")
            try:
                task = int(task)
            except ValueError:
                print(
                    f"The task must be a number from the following list: {self.accepted_tasks}"
                )
                continue

        print(f"Selected task {task}")

        self.cs_task = task
        self.publish()

    def publish(self):
        self.publisher_.publish(Int8(data=self.cs_task))


def main(args=None):
    rclpy.init(args=args)

    fake_cs_task_selector = FakeCSTaskSelector()

    while rclpy.ok():
        fake_cs_task_selector.select_task()

    fake_cs_task_selector.destroy_node()
    rclpy.shutdown()
