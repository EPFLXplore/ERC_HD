import cv2 as cv
import numpy as np
from vision.controlpanel.utils import top_left_hw_to_corners

# super class representing a control panel object


class CPO:
    """
    position 0 --- 1
             |     |
             3 --- 2
    """

    # Constructor method that initializes a new instance of the class with a given position.
    def __init__(
        self,
        corners_coords_2d: np.ndarray = None,
        top_left_corner: np.ndarray = None,
        height: int = None,
        width: int = None,
    ) -> None:
        if corners_coords_2d is None:
            self.corners_coords_2d = top_left_hw_to_corners(
                top_left_corner, height, width
            )
            self._width = width
            self._height = height
        else:
            # sets the position attribute to the provided position.
            self.corners_coords_2d = corners_coords_2d
            self._width = abs(corners_coords_2d[1][0] - corners_coords_2d[0][0])
            self._height = abs(corners_coords_2d[2][1] - corners_coords_2d[0][1])

        self.corners_coords_3d = np.array(
            [[coord[0], coord[1], 0.0] for coord in self.corners_coords_2d]
        )
        self._is_target = False

    # Getter method for the position attribute.
    def get_corners_coords_2d(self):
        return self.corners_coords_2d  # returns the position attribute.

    def get_corners_coords_3d(self):
        return self.corners_coords_3d

    def get_center_coords_3d(self):
        return self.corners_coords_3d.mean(axis=0)

    def get_width(self):
        return self._width

    def get_height(self):
        return self._height

    # A method to draw the point on the screen.
    # Currently, this method does nothing because it only contains a 'pass' statement.
    # This method should be overridden in a subclass that implements the drawing functionality.
    def draw(self, frame):
        if self._is_target:
            cv.polylines(
                frame,
                [np.array(self._projected_coords).astype(np.int32)],
                True,
                (255, 0, 0),
                4,
            )
        else:
            cv.polylines(
                frame,
                [np.array(self._projected_coords).astype(np.int32)],
                True,
                (0, 0, 255),
                4,
            )

    # A special method that returns a string representation of the object.
    # This method is automatically called when the object is converted to a string using the str() function.
    def __str__(self):
        return f"Position: {self.corners_coords_2d}"  # returns a string representation of the object.

    def target(self):
        self.is_target = True

    def untarget(self):
        self.is_target = False

    def set_projected_coord(self, projected):
        self._projected_coords = projected.astype(np.int32)

    def get_projected(self):
        return self._projected_coords
