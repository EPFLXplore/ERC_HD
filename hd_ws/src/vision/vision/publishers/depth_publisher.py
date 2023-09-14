from rclpy.node import Node

from std_msgs.msg import Int32
import numpy as np


class DepthPublisher(Node):
    CENTER_CROP_SIZE = 20
    CENTER_CROP_PIXEL_NB = CENTER_CROP_SIZE * CENTER_CROP_SIZE
    CENTER_HALF_CROP_SIZE = CENTER_CROP_SIZE // 2

    def __init__(self):
        super().__init__("depth_publisher")

        self.publisher_ = self.create_publisher(Int32, "HD/vision/depth", 10)
        self.get_logger().info("Depth (in mm) Publisher Created")

    def publish(self, depth_array: np.ndarray):
        depth = self.process(depth_array)
        if depth > 0:
            depth_msg = Int32(data=depth)
            self.publisher_.publish(depth_msg)

    def process(self, depth_array: np.ndarray) -> int:
        # print(depth_array.shape)
        # crop the center of the image
        depth_crop = depth_array[
            depth_array.shape[0] // 2
            - self.CENTER_HALF_CROP_SIZE : depth_array.shape[0] // 2
            + self.CENTER_HALF_CROP_SIZE,
            depth_array.shape[1] // 2
            - self.CENTER_HALF_CROP_SIZE : depth_array.shape[1] // 2
            + self.CENTER_HALF_CROP_SIZE,
        ]
        depth_crop = depth_crop.flatten()

        # count number of 0s
        nb_zeros = np.count_nonzero(depth_crop == 0)

        # sum the depth values
        sum_depth = np.sum(depth_crop)

        # compute the average depth
        # cast to float because of division of int by int
        if self.CENTER_CROP_PIXEL_NB == nb_zeros:
            return 0
        else:
            depth = sum_depth / (self.CENTER_CROP_PIXEL_NB - nb_zeros)

        # self._logger.info(f"depth: {int(depth):6d}")

        # cast back to int
        return int(depth)
