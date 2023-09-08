from vision.controlpanel.cpo import CPO
import cv2 as cv
from vision.controlpanel.utils import *

"""
position 0 --- 1
         |     |
         3 --- 2
"""


class Button(CPO):
    def __init__(self, top_left_corner, height, width, id=0, value=0, offset=0) -> None:
        corners = top_left_hw_to_corners(top_left_corner, height, width)
        super().__init__(corners)
        self._isOn = False  # the on position means we can see the small rectangle looking from the top
        self._id = id
        self._value = value
        self.is_target = False
        self.coords_2d_to_turn_off = (corners[0] + corners[1]) // 2
        self.coords_2d_to_turn_on = (corners[2] + corners[3]) // 2
        self.coordinates_2d_target = self.coords_2d_to_turn_on
        print(self.coordinates_2d_target)

    def get_id(self):
        return self._id

    def get_value(self):
        return self._value

    def get_isOn(self):
        return self._isOn

    def change_state(self):
        self._isOn = not self._isOn
        self.coordinates_2d_target = (
            self.coords_2d_to_turn_off if self._isOn else self.coords_2d_to_turn_on
        )

    def __str__(self):
        return (
            super().__str__()
            + f" isOn: {self._isOn} id: {self._id} value: {self._value}"
        )

    def get_length(self):
        return self._length

    def get_height(self):
        return self._height

    def get_width(self):
        return self._width

    def get_top_center(self):
        return self._top_center

    def get_bottom_center(self):
        return self._bottom_center

    def draw(self, frame):
        # print(f'type: {self._projected_coords.astype(int).dtype}')
        # print(f'coordinates: {self._projected_coords}')
        # cv.circle(frame, tuple(self.projected_target.astype(int)) , 2, (0, 255, 0), 10)

        if self.is_target:
            cv.polylines(
                frame, self._projected_coords.astype(int), True, (255, 0, 0), 4
            )
            cv.circle(
                frame, tuple(self.projected_target.astype(int)), 2, (0, 255, 0), 10
            )
        else:
            if self._isOn:
                cv.polylines(
                    frame, self._projected_coords.astype(int), True, (0, 255, 0), 4
                )
            if not self._isOn:
                cv.polylines(
                    frame, self._projected_coords.astype(int), True, (0, 0, 255), 4
                )

    def isTarget(self):
        return self.is_target

    def get_target_point(self):
        if self._isOn:
            return self.turn_off_target.reshape(1, 3)
        else:
            return self.turn_on_target.reshape(1, 3)

    def set_projected_coord(self, projected):
        super().set_projected_coord(projected)

        points = projected[0]
        if self._isOn:
            self.projected_target = (points[0] + points[1]) // 2
        else:
            self.projected_target = (points[2] + points[3]) // 2

    def get_target(self):
        res = np.zeros(3)
        res[:2] = self.coordinates_2d_target
        return res.astype(int)
