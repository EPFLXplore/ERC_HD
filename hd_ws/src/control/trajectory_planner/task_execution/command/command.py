import time
import math
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
import kinematics_utils.quaternion_arithmetic as qa
import kinematics_utils.pose_tracker as pt
import kinematics_utils.pose_corrector as pc
from collections.abc import Callable


class Command:
    """abstract class representing a command"""
    def __init__(self, executor=None):
        self.executor = executor
        self.execute_count = 0
        self.has_failed = False

    def createSetter(self, attribute: str):
        """create a setter for the given attribute"""
        def setter(val):
            setattr(self, attribute, val)

        if not hasattr(self, "set" + attribute.capitalize()):
            setattr(self, "set" + attribute.capitalize(), setter)

    def createSetters(self, *attributes):
        for attr in attributes:
            self.createSetter(attr)

    def execute(self):
        """attempts to execute command
        returns a bool indicating if it succeeded"""
        self.execute_count += 1

    def abort(self):
        """stops all movement"""

    def done(self):
        """indicate if command has executed correctly (by default returns true as soon as the commands has been executed once)"""
        return self.execute_count > 0

    def hasFailed(self):
        return self.has_failed
    