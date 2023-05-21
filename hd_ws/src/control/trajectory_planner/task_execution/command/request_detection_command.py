from task_execution.command.command import *


class RequestDetectionCommand(Command):
    def __init__(self, executor=None):
        super().__init__(executor)
        self.max_wait_time = 10
    
    def execute(self):
        super().execute()
        time.sleep(2)
        pt.deprecate_detection()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to leave release ressources
        while not pt.DETECTION_UPDATED:
            if time.time()-start > self.max_wait_time:
                return  # TODO: indicate that no detection was recorded (command failed)
            rate.sleep()
