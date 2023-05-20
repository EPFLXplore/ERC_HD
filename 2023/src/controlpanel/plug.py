from controlpanel.cpo import CPO
from controlpanel.utils import *


class Plug(CPO):
    def __init__(self, top_left_corner, height, width) -> None:
        position = get_coords(top_left_corner, height, width)
        super().__init__(position)
        self._is_plugged_in = False

    def get_is_plugged_in(self):
        return self._is_plugged_in
        
    def change_state(self):
        self._is_plugged_in = not self._is_plugged_in

        

    def get_target(self):
        res = np.zeros(3)
        res[:2] = self._position.sum() // 4
        return res.astype(int)