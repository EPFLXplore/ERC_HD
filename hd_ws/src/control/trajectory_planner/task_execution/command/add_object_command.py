from task_execution.command.command import *


class AddObjectCommand(Command):
    def __init__(self, executor=None, pose=None, shape=None, type="box", name="gustavo"):
        super().__init__(executor)
        self.pose = pose
        self.shape = shape
        self.type = type
        self.name = name
        self.createSetters("pose", "shape", "type", "name")
    
    def execute(self):
        super().execute()
        self.executor.addObjectToWorld(self.shape, self.pose, self.name, self.type)
        time.sleep(.5)

