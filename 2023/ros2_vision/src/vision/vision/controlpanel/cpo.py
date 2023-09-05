import cv2 as cv
import numpy as np

# super class representing a control panel object


class CPO:
    """
    position 0 --- 1
             |     |
             3 --- 2
    """

    # Constructor method that initializes a new instance of the class with a given position.
    def __init__(self, position):
        self._position = (
            position  # sets the position attribute to the provided position.
        )
        self._width = abs(position[1][0] - position[0][0])
        self._height = abs(position[2][1] - position[0][1])
        self._position_3D = np.array(
            [[coord[0]] + [coord[1]] + [0.0] for coord in position]
        )
        self._is_target = False

    # Getter method for the position attribute.
    def get_position(self):
        return self._position  # returns the position attribute.

    def get_3d_coords(self):
        return self._position_3D

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
        return f"Position: {self._position}"  # returns a string representation of the object.

    def target(self):
        self.is_target = True

    def untarget(self):
        self.is_target = False

    def set_projected_coord(self, projected):
        self._projected_coords = projected.astype(np.int32)

    def get_projected(self):
        return self._projected_coords
