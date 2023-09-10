from vision.controlpanel.panel import Panel
from vision.ar_tag import ARTag
from vision.controlpanel.button import Button
from cv2 import aruco
import numpy as np


# Class PanelA represents the central panel containing the buttons
class PanelA(Panel):
    name = "A"
    AR_SIZE = 49 # 50 # official measure
    BUTTON_HEIGHT = 42  # 50    # official measure
    BUTTON_WIDTH = 21   # 25    # official measure

    # origin is in the middle of ar tag, with x going to the right and y going up
    HORIZONTAL_DIST_BETWEEN_BUTTONS = 84 # mm
    VERTICAL_DIST_BETWEEN_BUTTONS = 71 # mm
    rows = {0: BUTTON_WIDTH,
            1: BUTTON_WIDTH - VERTICAL_DIST_BETWEEN_BUTTONS,
            2: BUTTON_WIDTH - 2 * VERTICAL_DIST_BETWEEN_BUTTONS,}
    cols = {0: -BUTTON_WIDTH,
            1: 0,
            2: HORIZONTAL_DIST_BETWEEN_BUTTONS - BUTTON_WIDTH,
            3: HORIZONTAL_DIST_BETWEEN_BUTTONS,}
    buttons_top_left_corner = [
        [cols[2], rows[0]],
        [cols[3], rows[0]],
        [cols[0], rows[1]],
        [cols[1], rows[1]],
        [cols[2], rows[1]],
        [cols[3], rows[1]],
        [cols[0], rows[2]],
        [cols[1], rows[2]],
        [cols[2], rows[2]],
        [cols[3], rows[2]],
    ]
    FRACTION = 0.5
    HORIZONTAL_CENTER_OFFSET = int(BUTTON_WIDTH / 2 * FRACTION)  # mm
    

    def __init__(self, camera_matrix, dist_coeffs, values):
        super().__init__()
        print(f'Button horizontal offset: {self.HORIZONTAL_CENTER_OFFSET}')
        self.ar_tag = ARTag(
            aruco.DICT_4X4_50, self.AR_SIZE, 2, camera_matrix, dist_coeffs
        )
        self.buttons = [
            Button(corner, self.BUTTON_HEIGHT, self.BUTTON_WIDTH, id, values)
            for id, corner in enumerate(self.buttons_top_left_corner)
        ]
        left = np.array([-self.HORIZONTAL_CENTER_OFFSET, 0])
        for button in self.buttons:
            button.coordinates_2d_target = button.coords_2d_to_turn_on + left
            left *= -1
        print("New offsets")
        for button in self.buttons:
            print(button.get_target())

        self.target = 0

    def get_possible_inputs(self):
        return "select one of the buttons going from 0 to 9\ntag   0-1\n2-3   4-5\n6-7   8-9"

    def set_target(self, target):
        self.buttons[self.target].untarget()
        self.target = target
        self.buttons[target].target()

    def detect_ar_tag(self, frame):
        result = self.ar_tag.detect(frame)
        self._detected = result
        return result

    # must be called after detect_ar_tag to be sure that the positions are not outdated or None
    def project(self):
        for button in self.buttons:
            projected_button = self.ar_tag.project(button.get_corners_coords_3d())
            button.set_projected_coord(projected_button)

    # must be called after detect_ar_tag to be sure that the positions are not outdated or None
    def draw(self, frame):
        self.ar_tag.draw(frame)
        for button in self.buttons:
            button.draw(frame)

    def get_target(self):
        return [self.buttons[self.target].get_target()] + self.ar_tag.get_vecs()
