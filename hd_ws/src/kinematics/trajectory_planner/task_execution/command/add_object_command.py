from task_execution.command.command import *
from hd_interfaces.msg import Object


class AddObjectCommand(Command):
    def __init__(self, executor=None, pose=None, shape=None, type=Object.BOX, operation=Object.ADD, name="gustavo"):
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

