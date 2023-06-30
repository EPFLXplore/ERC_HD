import rclpy
from rclpy.node import Node

from interfaces.msg import PanelObject

from std_msgs.msg import String
import geometry_msgs
from geometry_msgs.msg import Pose

class VisionPublisher(Node):

    def __init__(self):
        super().__init__('vision_distance_publisher')
        self.publisher_=self.create_publisher(PanelObject, '/HD/vision/distance_topic', 10)
        # timer_period = 1
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # # self.projector = projector
        # self.i = 0

    
    # def timer_callback(self):
    #     msg = PanelObject()

    #     msg.id = 1

    #     msg.pose = Pose()
    #     msg.pose.position.x = 1.0
    #     msg.pose.position.y = 1.0
    #     msg.pose.position.z = 1.0

    #     msg.pose.orientation.x = 2.0
    #     msg.pose.orientation.y = 1.0
    #     msg.pose.orientation.z = 1.0
    #     msg.pose.orientation.w = 1.0



    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%d"' % msg.id)
    #     self.get_logger().info('Publishing: "%d"' % msg.pose.orientation.x)

    #     self.i += 1

    
    def create_panelobject_message(self, id, pos_x, pos_y, pos_z, or_x, or_y, or_z, or_w):
        msg = PanelObject()

        msg.id = id

        msg.pose = Pose()

        msg.pose.position.x = pos_x
        msg.pose.position.y = pos_y
        msg.pose.position.z = pos_z

        msg.pose.orientation.x = or_x
        msg.pose.orientation.y = or_y
        msg.pose.orientation.z = or_z
        msg.pose.orientation.w = or_w

        return msg
    
    def publish_inform(self, msg):
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing msg with id: "%d"' % msg.id)
        # self.get_logger().info('Publishing msg with orientation x : "%f"' % msg.pose.orientation.x)


def main(args=None):
    rclpy.init(args=args)


    vision_publisher = VisionPublisher()
    vision_publisher.get_logger().info('BBBBBBBBBBBBBB')
    rclpy.spin(vision_publisher)

if __name__ == '__main__':
    main()

