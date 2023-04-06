import CPO
import cv2 as cv

"""
position 0 --- 1
         |     |
         2 --- 3
"""

class Button(CPO):
    def __init__(self, position, id, value, length) -> None:
        super().__init__(position)
        self._isOn = False
        self._id = id
        self._value = value
        self._top_center = position[0] + (self.get_width/2, self.get_height/4)
        self._bottom_center = position[0] + (self._width/2, 3 * (self._height/4))

    def get_id(self):
        return self._id
    
    def get_value(self):
        return self._value
    
    def get_isOn(self):
        return self._isOn
    
    def change_state(self):
        self._isOn = not self._isOn

    def __str__(self):
        return super().__str__() + f" isOn: {self._isOn} id: {self._id} value: {self._value}"
    
    def get_length(self):
        return self._length
    
    def get_height(self):
        return self._height
    
    def get_width(self):
        return self._width
    
    def get_top_center(self):
        return self._top_center
    
    def get_bottom_center(self):
        return self._bottom_center
    
    def draw_frame(self, frame):
        if self._isOn:
            cv.polylines(frame, self._position, 10, (0, 255, 0), 10)
        else:
            cv.polylines(frame, self._position, 10, (0, 0, 255), 10)

    # point coordinates need to be a tuple of ints
    def draw_point(self, frame, point):
        cv.circle(frame, point, 2, (0, 100, 255), 8)

    