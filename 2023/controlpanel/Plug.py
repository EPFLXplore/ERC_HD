import controlpanel.cpo as cpo

class Plug(cpo):
    def __init__(self, position) -> None:
        super().__init__(position)
        self._is_plugged_in = False

    def get_is_plugged_in(self):
        return self._is_plugged_in
    
    def change_state(self):
        self._is_plugged_in = not self._is_plugged_in

        

