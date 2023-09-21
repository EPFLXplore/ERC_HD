import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8


class FakeCSTaskSelector(Node):
    def __init__(self):
        super().__init__("fake_cs_task_selector")

        self.publisher_ = self.create_publisher(Int8, "ROVER/HD_element_id", 10)
        self.get_logger().info("Fake CS Task Selector Created")

        self.cs_task = 0
        self.id_to_task = self.setup_big_dict()

        self.valid_ids = set(self.id_to_task.keys())

    def setup_big_dict(self):
        id_to_task = {}
        id_to_task.update({100 + x: f"Turn ON  button ID: {x}" for x in range(10)})
        id_to_task.update({110 + x: f"Turn OFF button ID: {x}" for x in range(10)})
        id_to_task[10] = "Turn ON Big Button"
        id_to_task[13] = "Turn OFF Big Button"
        id_to_task[20] = "Voltmeter"
        id_to_task[21] = id_to_task[20]
        id_to_task[30] = "Ethernet Cable"
        id_to_task[31] = id_to_task[30]
        return id_to_task

    def select_task(self):
        task = -1
        while task not in self.valid_ids:
            task = input("Enter task number: ")
            try:
                task = int(task)
                if task not in self.valid_ids:
                    break       # to remove this break
                    print(f"The task number {task:3d} is not valid")
                    print("Try again")
                else:
                    print(f"Selected task {task:3d}:  {self.id_to_task[task]}")
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