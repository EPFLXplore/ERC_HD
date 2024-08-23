from numpy import ndarray


class ModuleInterface:
    def __init__(self, confg_file):
        raise NotImplementedError(
            "__init__ method should be implemented in the child class"
        )

    def __call__(self, rgb_frame: ndarray, depth_frame: ndarray):
        raise NotImplementedError(
            "__call__ method should be implemented in the child class"
        )
