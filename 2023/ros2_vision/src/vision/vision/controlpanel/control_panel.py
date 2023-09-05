from __future__ import annotations

from vision.controlpanel.panel_a import PanelA
from vision.controlpanel.panel_b1 import PanelB1
from vision.controlpanel.panel_b2 import PanelB2

import cv2 as cv


class ControlPanel:
    possible_panels = set(["A", "B1", "B2"])
    selected_panel = "A"
    KEY2PANEL = {ord("1"): "B1", ord("2"): "B2", ord("a"): "A"}

    def __init__(self, camera_matrix, dist_coeffs, button_values):
        self.panels = {
            "A": PanelA(camera_matrix, dist_coeffs, button_values),
            "B1": PanelB1(camera_matrix, dist_coeffs),
            "B2": PanelB2(camera_matrix, dist_coeffs),
        }

    # select the panel to be displayed
    def select_panel(self, panel):
        if panel in self.possible_panels:
            self.selected_panel = panel
        else:
            raise Exception(
                f"Panel {panel} does not exist, possible panels are {self.possible_panels}"
            )

    def draw(self, frame, draw_everything=False):
        cv.putText(
            frame,
            self.selected_panel,
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
            cv.LINE_AA,
        )

        if draw_everything:
            for panel in self.panels.values():
                panel.draw(frame)

        return self.get_selected_panel().draw(frame)

    def get_panel(self, panel):
        return self.panels[panel]

    def get_selected_panel(self):
        return self.panels[self.selected_panel]

    def update(self, frame):
        translation, rotation = self.get_selected_panel().update(frame)

    def __str__(self):
        return f"ControlPanel(possible_panels={self.possible_panels},\
                    selected_panel={self.selected_panel},\
                    panels={self.panels})"

    def get_possible_inputs(self):
        return self.get_selected_panel().get_possible_inputs()

    def set_target(self, target):
        if target == 0 or target == 10:
            self.select_panel("B1")
            if target == 0:
                self.get_selected_panel().set_target(0)
            else:
                self.get_selected_panel().set_target(1)

        if target == 30:
            self.select_panel("B2")

        if target >= 20 and target <= 29:
            self.select_panel("A")
            self.get_selected_panel().set_target(target - 20)

    def detect_ar_tag(self, frame):
        return self.get_selected_panel().detect_ar_tag(frame)

    def project(self):
        return self.get_selected_panel().project()

    def get_target(self):
        # print(self.get_selected_panel().get_target())
        return self.get_selected_panel().get_target()
