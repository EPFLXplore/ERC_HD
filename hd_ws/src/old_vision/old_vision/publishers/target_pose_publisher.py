from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from hd_interfaces.msg import TargetInstruction


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("target_pose_publisher")

        self.publisher_ = self.create_publisher(TargetInstruction, "target_pose", 10)
        self.get_logger().info("Target Pose Publisher Created")

    def timer_callback(self):
        pose = Pose()
        pose.position.x = 0.5

        self.publisher_.publish(pose)

    def publish(self, translation, quaternion):
        translation_dict = {
            "x": translation[0],
            "y": translation[1],
            "z": translation[2],
        }
        quaternion_dict = {
            "x": quaternion[0],
            "y": quaternion[1],
            "z": quaternion[2],
            "w": quaternion[3],
        }

        point = Point(**translation_dict)
        quaternion = Quaternion(**quaternion_dict)

        pose1 = Pose()
        pose1.position = point
        pose1.orientation = quaternion

        pose2 = Pose()
        pose2.position = Point(x=0.0, y=0.0, z=0.0)

        msg = TargetInstruction()
        msg.ar_tag_pose = pose1
        msg.object_pose = pose2

        self.publisher_.publish(msg)
