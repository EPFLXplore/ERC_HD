from task_execution.command.command import *
from hd_interfaces.msg import Object
from typing import List


class AddObjectCommand(Command):
    """Command for adding, modifying or removing an object in the MoveIt world"""
    # TODO: modify the name to manipulate instead of add

    def __init__(self, executor=None, pose: Pose=None, shape: List[float]=None, type: int=Object.BOX, operation: int=Object.ADD, name: str="gustavo"):
        super().__init__(executor)
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

