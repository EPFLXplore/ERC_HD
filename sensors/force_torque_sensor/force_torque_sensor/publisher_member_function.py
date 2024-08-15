# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray #String 
#from bota_serial_example import * 
from py_pubsub.bota_serial_example import *

class MinimalPublisher(Node):

    def __init__(self, bota):
        super().__init__('minimal_publisher')
        self.bota = bota
        self.publisher_ = self.create_publisher(Float32MultiArray , 'topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float32MultiArray()
        
        msg.data = self.bota._data 
        self.get_logger().info('Publishing: "%s"' % self.bota._data)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        print()
        self.i += 1


def main(args=None):

    bota = BotaSerialSensor()
    bota.run() # Starts the thread

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(bota)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

    # Close the thread 
    bota._pd_thread_stop_event.set()
    bota.proc_thread.join()
    #check_thread.join()
    bota._ser.close()

if __name__ == '__main__':
    main()
