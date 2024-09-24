# TODO: move this to another global utility package

import rclpy
from rclpy.node import Node
import threading
from typing import Optional


class NodeTemplate(Node):
    def __init__(self, name: str = ""):
        super().__init__(name)
        self.create_ros_interfaces()

    def create_ros_interfaces(self):
        pass
    
    def loop(self, hz: int = 10):
        rate = self.create_rate(hz)
        while rclpy.ok():
            rate.sleep()
    
    @classmethod
    def main(cls, hz: int = 10, args: Optional[list[str]] = None):
        rclpy.init(args=args)
        node = cls()

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
        thread.start()

        try:
            node.loop(hz)
        except KeyboardInterrupt:
            pass
        
        rclpy.shutdown()
        thread.join()
