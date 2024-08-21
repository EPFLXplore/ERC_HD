from .modules.perception_module_interface import PerceptionModuleInterface


class ModuleManager:
    def __init__(self):
        self.modules = []

    def add_module(self, module: PerceptionModuleInterface):
        self.modules.append(module)

    def remove_module(self, module: PerceptionModuleInterface):
        self.modules.remove(module)

    def process_rgb(self, rgb_frame):
        for module in self.modules:
            module.process_rgb(rgb_frame)

    def process_depth(self, depth_frame):
        for module in self.modules:
            module.process_depth(depth_frame)

    def post_process(self, rgb_frame, depth_frame):
        for module in self.modules:
            module.post_process(rgb_frame, depth_frame)

    # def post_process(self, rgb_frame, depth_frame):
    #     rock_masks = None
    #     for module in self.modules:
    #         if isinstance(module, RockDetectionModule):
    #             rock_masks = (
    #                 module.rock_masks
    #             )  # Capture rock masks from the rock detection module

    #     for module in self.modules:
    #         if isinstance(module, RockSizeEstimationModule):
    #             module.post_process(rock_masks, depth_frame)
