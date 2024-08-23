from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from rclpy.node import Node

from std_msgs.msg import Int8


class SimplePosePublisher(Node):
    def __init__(self):
        super().__init__("simple_target_pose_publisher")

        self.publisher_ = self.create_publisher(
            Pose, "HD/vision/simple_target_pose", 10
        )
        self.get_logger().info("Target Pose Publisher Created")

    def timer_callback(self):
        pose = Pose()
        pose.position.x = 0.5

        self.publisher_.publish(pose)

    def publish(self, translation, quaternion):
        pose = self.toPose(translation, quaternion)
        # print(
        #     f"difference: x:{pose.position.x - ar_tag_pose.position.x}, y: {pose.position.y - ar_tag_pose.position.y}, z: {pose.position.z - ar_tag_pose.position.z}"
        # )

        self.publisher_.publish(pose)

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
