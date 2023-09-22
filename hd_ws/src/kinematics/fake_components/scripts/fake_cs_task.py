#!/usr/bin/env python3

import rclpy
from hd_interfaces.msg import Task
from std_msgs.msg import Int8


def main():
    rclpy.init()
    node = rclpy.create_node("fake_cs_task")

    task_pub = node.create_publisher(Int8, "/ROVER/element_id", 10)

    msg = Int8()
    msg.data = 0
    task_pub.publish(msg)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
