from .pose_msg import PoseMsg
from custom_msg.msg import ArucoObject
from std_msgs.msg import Int8


class TargetInstructionMsg:

    @staticmethod
    def create_message(
        aruco_translation,
        aruco_quaternion,
        target_translation,
        target_quaternion,
        target_id,
    ) -> ArucoObject:
        aruco_msg = PoseMsg.create_message(aruco_translation, aruco_quaternion)
        target_msg = PoseMsg.create_message(target_translation, target_quaternion)

        target_instruction = ArucoObject()
        target_instruction.ar_tag_pose = aruco_msg
        target_instruction.object_pose = target_msg
        # target_instruction.task_id = Int8(data=target_id)
        target_instruction.is_detected = True
        return target_instruction

    @staticmethod
    def empty_message() -> ArucoObject:
        target_instruction = ArucoObject()
        target_instruction.is_detected = False
        return target_instruction
