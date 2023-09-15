from vision.controlpanel.panel import Panel
from vision.controlpanel.plug import Plug
from vision.ar_tag import ARTag
from vision.controlpanel.button import Button
from cv2 import aruco


class PanelB1(Panel):
    name = "B1"
    AR_SIZE = 32
    BUTTON_SIZE = 50  # not sure from the rules not explicit

    def __init__(self, camera_matrix, dist_coeffs):
        super().__init__()
        self.ar_tag_top = ARTag(
            aruco.DICT_4X4_50, self.AR_SIZE, 1, 14, camera_matrix, dist_coeffs
        )
        self.ar_tag_bottom = ARTag(
            aruco.DICT_4X4_50, self.AR_SIZE, 0, 15, camera_matrix, dist_coeffs
        )
        self.button = Button(
            [-62 - self.BUTTON_SIZE // 2, self.BUTTON_SIZE // 2],
            self.BUTTON_SIZE,
            self.BUTTON_SIZE,
        )
        self.plug = Plug(
            [-62 - self.BUTTON_SIZE // 2, self.BUTTON_SIZE // 2],
            self.BUTTON_SIZE,
            self.BUTTON_SIZE,
        )

    def get_possible_inputs(self):
        return "select target\nElectric plug: 0\nButton       : 1"

    def set_target(self, target):
        if target == 0:
            self.button.untarget()
            self.plug.target()
        elif target == 10 or target == 13:
            if target == 10:
                self.button.target(to_turn_on=True)
            else:
                self.button.target(to_turn_on=False)
            self.plug.untarget()
        else:
            # throw exception
            raise Exception(f"target {target} not valid, valid targets are 0 and 1")

    def detect_ar_tag(self, frame):
        if self.button.isTarget():
            return self.ar_tag_top.detect(frame)
        else:
            return self.ar_tag_bottom.detect(frame)

    # must be called after detect_ar_tag to be sure that the positions are not outdated or None
    def project(self):
        if self.button.isTarget():
            projected_button = self.ar_tag_top.project(
                self.button.get_corners_coords_3d()
            )
            self.button.set_projected_coord(projected_button)
        else:
            projected_plug = self.ar_tag_bottom.project(
                self.plug.get_corners_coords_3d()
            )
            self.plug.set_projected_coord(projected_plug)

    def draw(self, frame):
        if self.button.isTarget():
            self.ar_tag_top.draw(frame)
            self.button.draw(frame)
        else:
            self.ar_tag_bottom.draw(frame)
            self.plug.draw(frame)

    def get_target(self):
        if self.button.isTarget():
            return [self.button.get_target()] + self.ar_tag_top.get_vecs()
        else:
            return [self.plug.get_target()] + self.ar_tag_bottom.get_vecs()