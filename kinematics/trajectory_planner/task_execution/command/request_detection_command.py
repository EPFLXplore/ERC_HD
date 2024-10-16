from task_execution.command.command import *


class RequestDetectionCommand(Command):
    def __init__(self):
        super().__init__()
        self.preliminary_wait_duration = 2
        self.timeout = 10
    
    def execute(self):
        super().execute()
        time.sleep(self.preliminary_wait_duration)
        pt.deprecate_detection()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to release ressources
        while not self.executor.detectionUpdated():
            if time.time()-start > self.timeout:
                self.has_failed = True
                return
            rate.sleep()


class RequestArucoDetectionCommand(Command):
    def __init__(self):
        super().__init__()
        self.preliminary_wait_duration = 2
        self.timeout = 10
    
    def execute(self):
        super().execute()
        time.sleep(self.preliminary_wait_duration)
        pt.perception_tracker.aruco_object_detection.deprecate()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to release ressources
        while pt.perception_tracker.aruco_object_detection.deprecated:
            if time.time()-start > self.timeout:
                self.has_failed = True
                return
            rate.sleep()


class RequestRockDetectionCommand(Command):
    def __init__(self):
        super().__init__()
        self.preliminary_wait_duration = 2
        self.timeout = 10
    
    def execute(self):
        super().execute()
        time.sleep(self.preliminary_wait_duration)
        pt.perception_tracker.rock_detection.deprecate()
        start = time.time()
        rate = self.executor.create_rate(25)    # 25 hz rate in order to release ressources
        while pt.perception_tracker.rock_detection.deprecated:
            if time.time()-start > self.timeout:
                self.has_failed = True
                return
            rate.sleep()
