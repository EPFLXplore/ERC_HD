from task_execution.command.command import *


class RequestHumanVerification(Command):
    def __init__(self, timeout: float = 5.0):
        super().__init__()
        self.timeout = timeout  # not used yet

    def execute(self):
        super().execute()
        success = self.executor.requestHumanVerification()
        if not success:
            self.has_failed
