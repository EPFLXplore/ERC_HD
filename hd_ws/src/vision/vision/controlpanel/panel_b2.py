from vision.controlpanel.panel import Panel
from vision.controlpanel.ethernet_plug import Ethernet_plug
from vision.ar_tag import ARTag
from cv2 import aruco


class PanelB2(Panel):
    name = "B2"
    AR_SIZE = 50

    def __init__(self, camera_matrix, dist_coeffs):
        super().__init__()
        self.ar_tag = ARTag(
            aruco.DICT_4X4_50, self.AR_SIZE, 3, 13, camera_matrix, dist_coeffs
        )
        self.ether_plug = Ethernet_plug(
            [-self.AR_SIZE // 2, 71 + self.AR_SIZE // 2], self.AR_SIZE, self.AR_SIZE
        )

    def detect_ar_tag(self, frame):
        result = self.ar_tag.detect(frame)
        self._detected = result
        return result

    # must be called after detect_ar_tag to be sure that the positions are not outdated or None
    def project(self):
        prjected_plug = self.ar_tag.project(self.ether_plug.get_corners_coords_3d())
        self.ether_plug.set_projected_coord(prjected_plug)

    # must be called after detect_ar_tag to be sure that the positions are not outdated or None
    def draw(self, frame):
        self.ar_tag.draw(frame)
        self.ether_plug.draw(frame)

    def get_target(self):
        return [self.ether_plug.get_target()] + self.ar_tag.get_vecs()
