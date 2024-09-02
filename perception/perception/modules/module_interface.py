from numpy import ndarray
from abc import ABC, abstractmethod

from rclpy.node import Node 


class ModuleInterface(ABC):
    def __init__(self, confg_file, node: Node) -> None:
        raise NotImplementedError(
            "__init__ method should be implemented in the child class"
        )

    def __call__(self, rgb_frame: ndarray, depth_frame: ndarray):
        """
        processes the frames, depending on module uses either both or only one of them ( rgb or depth )
        stores the current result in a attribute used for drawing
        """
        raise NotImplementedError(
            "__call__ method should be implemented in the child class"
        )

    def draw(frame: ndarray) -> None:
        """
        frame is the rgb image comming from the camera
        Draws the results on from __call__ on the frame if any 
        Must always be called after __call__

        Returns None as the frame is modified
        """
        raise NotImplementedError(
            "draw method should be implemented in the child class"
        )
