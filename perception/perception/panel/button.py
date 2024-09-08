import cv2 as cv
import numpy as np
from ..geometry import transform_and_project

"""
position 0 --- 1
         |     |
         3 --- 2
"""


class Button:

    def __init__(
        self,
        height,
        width,
        horizontal_offset,
        vertical_offset,
        relative_position,
        click_offset_fraction,
        camera_matrix,
    ):
        self.height = height
        self.width = width
        self.horizontal_offset = horizontal_offset
        self.vertical_offset = vertical_offset
        self.coordinates = self._init_coordinates()
        self.relative_position = relative_position
        self.click_offset_fraction = click_offset_fraction
        self.top_click_pos, self.bottom_click_pos = self._init_click_positions()
        self.target = "no"  # one of 'no', 'top', 'bottom'
        self.camera_matrix = camera_matrix

    def draw(self, frame, transform):
        # if self.is_target:
        projected_coords = self.project(transform)
        # print(f"Projected button coordinates: {projected_coords}")
        if self.target == "no":
            button_color = (255, 0, 0)
        else:
            button_color = (255, 0, 255)
        cv.polylines(frame, [projected_coords.astype(int)], True, button_color, 4)

        if self.target == "top":
            projected_target_point = transform_and_project(
                self.top_click_pos, transform, self.camera_matrix
            )
        if self.target == "bottom":
            projected_target_point = transform_and_project(
                self.bottom_click_pos, transform, self.camera_matrix
            )
        if self.target == "top" or self.target == "bottom":
            cv.circle(
                frame,
                tuple(projected_target_point.astype(int)),
                2,
                (0, 255, 0),
                10,
            )

    # else:
    #     if self._isOn:
    #         cv.polylines(
    #             frame, self._projected_coords.astype(int), True, (0, 255, 0), 4
    #         )
    #     if not self._isOn:
    #         cv.polylines(
    #             frame, self._projected_coords.astype(int), True, (0, 0, 255), 4
    #         )

    def project(self, transform):
        projected_points = []
        for point in self.coordinates:
            projected_points.append(
                transform_and_project(point, transform, self.camera_matrix)
            )
        return np.array(projected_points)

    def get_target(self):
        if self.target == "no":
            return "no"
        if self.target == "top":
            return self.top_click_pos
        if self.target == "bottom":
            return self.bottom_click_pos
        raise ValueError(f"Invalid target value: {self.target}")

    def _init_coordinates(self):
        horizontal_offset_3d = np.array([self.horizontal_offset, 0, 0])
        vectical_offset_3d = np.array([0, self.vertical_offset, 0])

        width_3d = np.array([self.width, 0, 0])
        height_3d = np.array([0, self.height, 0])

        top_left = horizontal_offset_3d + vectical_offset_3d

        top_right = top_left + width_3d
        bottom_left = top_left - height_3d
        bottom_right = top_left + width_3d - height_3d
        return np.array([top_left, top_right, bottom_right, bottom_left])

    def _init_click_positions(self):
        top_center = (self.coordinates[0] + self.coordinates[1]) / 2
        bottom_center = (self.coordinates[2] + self.coordinates[3]) / 2
        if self.relative_position == "right":
            offset = self.width * self.click_offset_fraction
        if self.relative_position == "left":
            offset = -self.width * self.click_offset_fraction
        top_center[0] += offset
        bottom_center[0] += offset
        return top_center, bottom_center
