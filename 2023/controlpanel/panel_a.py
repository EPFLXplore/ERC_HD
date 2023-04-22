from controlpanel.panel import Panel
from ar_tag import ARTag
from controlpanel.button import Button
from cv2 import aruco

# Class PanelA represents the central panel containing the buttons
class PanelA( Panel):
    name = 'A'
    AR_SIZE = 50
    BUTTON_HEIGHT = 50
    BUTTON_WIDTH = 25
    buttons_top_left = [ [84 - 25, -25],
                       [84, -25],
                       [-25, 71 - 25],
                       [0, 71 - 25],
                       [ 84 - 25, 71 - 25],
                       [84, 71 - 25],
                       [-25, 2 * 71 - 25 ],
                       [0, 2 * 71 - 25],
                       [84 - 25, 2 * 71 - 25],
                       [84, 2 * 71 - 25],
                          ]
    

    def __init__(self, camera_matrix, dist_coeffs, values):
        super().__init__()
        self.ar_tag = ARTag(aruco.DICT_4X4_50, self.AR_SIZE , camera_matrix, dist_coeffs)
        self.buttons = [ Button(corner, id, values,
                                 self.BUTTON_HEIGHT, self.BUTTON_WIDTH)
                                   for id, corner  in enumerate(self.buttons_top_left)]
        

    def draw(self, frame):
        # self.ar_tag.draw(frame)
        for button in self.buttons:
            button.draw(frame)
        