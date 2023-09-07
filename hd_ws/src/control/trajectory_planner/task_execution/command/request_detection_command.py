from task_execution.command.command import *


class RequestDetectionCommand(Command):
    def __init__(self, executor=None):
        super().__init__(executor)
        self.preliminary_wait_duration = 2
        self.max_wait_duration = 10
    
    def execute(self):
        super().execute()
        time.sleep(self.preliminary_wait_duration)
        pt.deprecate_detection()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to leave release ressources
        while not pt.DETECTION_UPDATED:
            if time.time()-start > self.max_wait_duration:
                self.has_failed = True
                return  # TODO: indicate that no detection was recorded (command failed)
            rate.sleep()
