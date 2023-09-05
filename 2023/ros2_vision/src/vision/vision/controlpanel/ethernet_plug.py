from vision.controlpanel.plug import Plug


class Ethernet_plug(Plug):
    def __init__(self, top_left_corner, height, width) -> None:
        super().__init__(top_left_corner, height, width)
