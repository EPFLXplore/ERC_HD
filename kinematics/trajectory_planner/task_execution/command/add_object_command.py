from task_execution.command.command import *
from custom_msg.msg import Object
from typing import List


class AddObjectCommand(Command):
    """Command for adding, modifying or removing an object in the MoveIt world"""
    # TODO: modify the name to manipulate instead of add

    def __init__(self, pose: Pose=None, shape: List[float]=None, type: int=Object.BOX, operation: int=Object.ADD, name: str="gustavo"):
        super().__init__()
        if pose is None:
            pose = Pose()
        if shape is None:
            shape = [0.0, 0.0, 0.0]
        self.pose = pose
        self.shape = shape
        self.type = type
        self.operation = operation
        self.name = name
        self.createSetters("pose", "shape", "type", "name")
    
    def execute(self):
        super().execute()
        self.executor.addObjectToWorld(self.shape, self.pose, self.name, self.type, self.operation)
        time.sleep(.5)


class AttachObjectCommand(Command):
    def __init__(self, pose: Pose=None, shape: List[float]=None, type: int=Object.BOX, operation: int=Object.ADD, name: str="gustavo"):
        super().__init__()
        if pose is None:
            pose = Pose()
        if shape is None:
            shape = [0.0, 0.0, 0.0]
        self.pose = pose
        self.shape = shape
        self.type = type
        self.operation = operation
        self.name = name
        self.createSetters("pose", "shape", "type", "name")
    
    def execute(self):
        super().execute()
        self.executor.attachObjectToGripper(self.shape, self.pose, self.name, self.type, self.operation)
        if self.operation == Object.REMOVE:
            time.sleep(.5)
            self.executor.addObjectToWorld(self.shape, self.pose, self.name, self.type, self.operation)
        time.sleep(.5)

