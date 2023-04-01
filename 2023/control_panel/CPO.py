
# super class representing a control panel object

class CPO:
    # Constructor method that initializes a new instance of the class with a given position.
    def __init__(self, position):
        self._position = position  # sets the position attribute to the provided position.

    # Getter method for the position attribute.
    def get_position(self):
        return self._position  # returns the position attribute.

    # A method to draw the point on the screen.
    # Currently, this method does nothing because it only contains a 'pass' statement.
    # This method should be overridden in a subclass that implements the drawing functionality.
    def draw(self):
        pass

    # A special method that returns a string representation of the object.
    # This method is automatically called when the object is converted to a string using the str() function.
    def __str__(self):
        return f"Position: {self._position}"  # returns a string representation of the object.

