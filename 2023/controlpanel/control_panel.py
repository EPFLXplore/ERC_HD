from controlpanel.panel_a import PanelA
from controlpanel.panel_b1 import PanelB1
from controlpanel.panel_b2 import PanelB2

class ControlPanel:
    possible_panels = set(['A', 'B1', 'B2'])
    selected_panel = 'A'
    KEY2PANEL = {
                ord('1'): 'B1',
                ord('2'): 'B2',
                ord('a'): 'A'
                }

    def __init__(self, camera_matrix, dist_coeffs, button_values):
        self.panels = {'A': PanelA(camera_matrix, dist_coeffs, button_values),
                        'B1': PanelB1(),
                        'B2': PanelB2()}


    # select the panel to be displayed
    def select_panel(self, panel):
        panel = self.KEY2PANEL[panel]
        if panel in self.possible_panels:
            self.selected_panel = panel
        else:
            raise Exception(f'Panel {panel} does not exist, possible panels are {self.possible_panels}')
        

    def draw(self, frame, draw_everything=False):
        if(draw_everything):
            for panel in self.panels.values():
                panel.draw(frame)

        return self.panels[self.selected_panel].draw(frame)
    

    def get_panel(self, panel):
        return self.panels[panel]
    

    

    def __str__(self):
        return f"ControlPanel(possible_panels={self.possible_panels},\
                    selected_panel={self.selected_panel},\
                    panels={self.panels})"