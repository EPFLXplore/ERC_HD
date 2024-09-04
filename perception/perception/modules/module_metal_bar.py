from .module_interface import ModuleInterface
import pyrealsense2 as rs
import numpy as np
import cv2
#from ultralytics import YOLO
import torch
from scipy.spatial.transform import Rotation as R  # Use scipy for quaternion operations
import rclpy

'''
PURPOSE OF THE MODULE
take rgb_frame, color_frame
return: a list of rocks with their positions (quaternions), max dimensions


'''
# Check if CUDA is available
cuda_available = torch.cuda.is_available()
device = torch.device("cuda" if cuda_available else "cpu")

class ModuleMetalBar(ModuleInterface):
    def __init__(self) -> None:
        raise NotImplementedError(
            "__init__ method should be implemented in the child class"
        )

