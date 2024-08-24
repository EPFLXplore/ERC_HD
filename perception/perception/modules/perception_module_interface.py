from abc import ABC, abstractmethod


class PerceptionModuleInterface(ABC):
    @abstractmethod
    def process_rgb(self, rgb_frame):
        pass

    @abstractmethod
    def process_depth(self, depth_frame):
        pass

    @abstractmethod
    def process_rgbd(self, rgb_frame, depth_frame):
        pass

    def post_process(self, *args):
        """Optional post-processing step"""
        pass
