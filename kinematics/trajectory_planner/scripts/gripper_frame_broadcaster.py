#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('gripper_frame_broadcaster')
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'hd_link6_1'
       t.child_frame_id = 'gripper'
       t.transform.translation.x = -0.2
       t.transform.translation.y = 0.0
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
