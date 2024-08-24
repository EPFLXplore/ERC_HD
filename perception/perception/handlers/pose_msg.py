from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class PoseMsg:

    @staticmethod
    def create_message(translation, quaternion) -> Pose:
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

        pose = Pose()
        pose.position = Point(**translation_dict)
        pose.orientation = Quaternion(**quaternion_dict)
        return pose

    @staticmethod
    def decode_message(pose: Pose):
        return (pose.position.x, pose.position.y, pose.position.z), (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
