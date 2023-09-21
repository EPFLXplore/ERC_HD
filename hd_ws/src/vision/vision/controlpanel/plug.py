from vision.controlpanel.cpo import CPO
from vision.controlpanel.utils import top_left_hw_to_corners
import numpy as np


class Plug(CPO):
    def __init__(self, top_left_corner, height, width) -> None:
        super().__init__(
            top_left_corner=top_left_corner,
            height=height,
            width=width,
        )
        print(f"plug: {self.get_center_coords_3d()}")
        self._is_plugged_in = False

    def get_is_plugged_in(self):
        return self._is_plugged_in

    def change_state(self):
        self._is_plugged_in = not self._is_plugged_in

    def get_target(self):
        res = np.zeros(3)
        res[:2] = self.get_corners_coords_2d().mean(axis=0)
        return res.astype(int)
