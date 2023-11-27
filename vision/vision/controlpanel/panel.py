# class Panel represents one of the 3 panels on the control panel.
class Panel:
    # values are from ERC 2023 rule book
    min_height = 500
    height = 400
    
    def __init__(self) -> None:
        self.max_height = self.min_height + self.height 

