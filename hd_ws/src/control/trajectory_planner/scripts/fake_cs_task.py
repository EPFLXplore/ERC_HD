#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from kerby_interfaces.msg import Task


def main():
    rclpy.init()
    node = rclpy.create_node("fake_cs_task")

    task_pub = node.create_publisher(Task, "/HD/task_assignment", 10)

    msg = Task()
    msg.id = 0
    msg.description = "btn"
    task_pub.publish(msg)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
