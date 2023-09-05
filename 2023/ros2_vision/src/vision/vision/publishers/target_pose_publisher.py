from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from rclpy.node import Node

from std_msgs.msg import Int8
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

    def publish(self, translation, quaternion, ar_translation, ar_quaternion, task):
        pose = self.toPose(translation, quaternion)
        ar_tag_pose = self.toPose(ar_translation, ar_quaternion)
        task = Int8(data=task)

        instruction = TargetInstruction()
        instruction.object_pose = pose
        instruction.ar_tag_pose = ar_tag_pose
        instruction.task_id = task
        self.publisher_.publish(instruction)

    def toPose(self, translation, quaternion) -> Pose:
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

        pose = Pose()
        pose.position = point
        pose.orientation = quaternion
        return pose
