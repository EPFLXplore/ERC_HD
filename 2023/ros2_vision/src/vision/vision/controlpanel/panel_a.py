from vision.controlpanel.panel import Panel
from vision.ar_tag import ARTag
from vision.controlpanel.button import Button
from cv2 import aruco
import numpy as np


# Class PanelA represents the central panel containing the buttons
class PanelA(Panel):
    name = "A"
    AR_SIZE = 50
    BUTTON_HEIGHT = 50
    BUTTON_WIDTH = 25

    # origin is in the middle of ar tag
    rows = {0: 25, 1: 25 - 71, 2: 25 - 2 * 71}
    cols = {0: -25, 1: 0, 2: 84 - 25, 3: 84}
    buttons_top_left = [
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

    def __init__(self, camera_matrix, dist_coeffs, values):
        super().__init__()
        self.ar_tag = ARTag(
            aruco.DICT_4X4_50, self.AR_SIZE, 2, camera_matrix, dist_coeffs
        )
        self.buttons = [
            Button(corner, self.BUTTON_HEIGHT, self.BUTTON_WIDTH, id, values)
            for id, corner in enumerate(self.buttons_top_left)
        ]
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
            projected_button = self.ar_tag.project(button.get_3d_coords())
            button.set_projected_coord(projected_button)

    # must be called after detect_ar_tag to be sure that the positions are not outdated or None
    def draw(self, frame):
        # self.ar_tag.draw(frame)
        for button in self.buttons:
            button.draw(frame)

    def get_target(self):
        return [self.buttons[self.target].get_target()] + self.ar_tag.get_vecs()
