from geometry_msgs.msg import Pose
from rclpy.node import Node


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("target_pose_publisher")

        self.publisher_ = self.create_publisher(Pose, "target_pose", 10)
        self.get_logger().info("Target Pose Publisher Created")

        timer_period = 0.5

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pose = Pose()
        pose.position.x = 0.5

        self.publisher_.publish(pose)
