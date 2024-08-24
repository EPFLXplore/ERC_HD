from numpy import ndarray
from abc import ABC, abstractmethod


class ModuleInterface(ABC):
    def __init__(self, confg_file) -> None:
        raise NotImplementedError(
            "__init__ method should be implemented in the child class"
        )

    def __call__(self, rgb_frame: ndarray, depth_frame: ndarray):
        raise NotImplementedError(
            "__call__ method should be implemented in the child class"
        )

    def draw(frame: ndarray) -> None:
        raise NotImplementedError(
            "draw method should be implemented in the child class"
        )
