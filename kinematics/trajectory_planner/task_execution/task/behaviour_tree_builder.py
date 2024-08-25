from __future__ import annotations
from task import *
import pygame
from math import sqrt, sin, cos, asin, acos, pi
from typing import Any, Union, Tuple, List
import sys
import random


Scalar = Union[
    int
]  # TODO: add support for numpy scalar types (and potentially other scalar types as well)
Array = Union[Tuple[Scalar, ...], List[Scalar]]
array_types_checkable = (
    tuple,
    list,
)  # for instance checking since subscripted generics such as List[Scalar] cannot be used for instance checking


def is_point_like(x: Any) -> bool:
    """
    Verify if x is convertible into Point2d
    """
    if isinstance(x, Point2d):
        return True
    if not isinstance(x, array_types_checkable):
        return False
    return len(x) == 2 and all(isinstance(v, Scalar) for v in x)


class Point2d:
    def __init__(self, x: Scalar = 0, y: Scalar = 0):
        self.x = x
        self.y = y

    @classmethod
    def make(cls, value: PointLike) -> Point2d:
        """
        Convert a PointLike to Point2d
        """
        if isinstance(value, cls):
            return value
        if isinstance(value, array_types_checkable):
            return cls(x=value[0], y=value[1])
        raise TypeError(
            f"Unsuported type '{type(value).__name__}' for conversion to Point2d"
        )

    @classmethod
    def eq(cls, p1: PointLike, p2: PointLike, precision: float = 10**-10) -> bool:
        """
        Check whether the two points are equal within some margin of error
        (the precision margin is on the squared norm)
        """
        return (cls.make(p1) - cls.make(p2)).sq_norm() <= precision

    @property
    def xy(self) -> Tuple[Scalar, Scalar]:
        return self.x, self.y

    def __getitem__(self, idx: int):
        if idx == 0:
            return self.x
        if idx == 1:
            return self.y
        raise ValueError

    def __add__(self, other: PointLike) -> Point2d:
        # TODO: maybe add more direct typechecking via is_point_like (for now type checking is done indirectly in the make method), and same for __sub__ and __rsub__ methods
        other = Point2d.make(other)
        return Point2d(x=self.x + other.x, y=self.y + other.y)

    def __radd__(self, other: PointLike) -> Point2d:
        return self + other

    def __sub__(self, other: PointLike) -> Point2d:
        other = Point2d.make(other)
        return Point2d(x=self.x - other.x, y=self.y - other.y)

    def __rsub__(self, other: PointLike) -> Point2d:
        other = Point2d.make(other)
        return Point2d(x=other.x - self.x, y=other.y - self.y)

    def __mul__(self, other: Union[Scalar, PointLike]) -> Point2d:
        if isinstance(other, (int, float)):
            return Point2d(x=other * self.x, y=other * self.y)
        if is_point_like(other):
            return self.dot(other)
        raise TypeError(
            f"Unsuported operand types for *: '{Point2d.__name__}' and '{type(other).__name__}'"
        )

    def __rmul__(self, other: Union[Scalar, PointLike]) -> Point2d:
        try:
            return self * other
        except TypeError:
            raise TypeError(
                f"Unsuported operand types for *: '{type(other).__name__}' and '{Point2d.__name__}'"
            )

    def __truediv__(self, other) -> Point2d:
        return Point2d(x=self.x / other, y=self.y / other)

    def __neg__(self) -> Point2d:
        return Point2d(x=-self.x, y=-self.y)

    def dot(self, other: PointLike) -> Scalar:
        other = Point2d.make(other)
        return self.x * other.x + self.y * other.y

    def sq_norm(self) -> float:
        return self.x**2 + self.y**2

    def norm(self) -> float:
        return sqrt(self.sq_norm())
    
    def __le__(self, other: PointLike) -> bool:
        other = Point2d.make(other)
        return self.x <= other.x and self.y <= other.y

    def __abs__(self) -> float:
        return self.norm()

    def normalized(self) -> Point2d:
        n = self.norm()
        if n == 0:
            return Point2d(x=0.0, y=0.0)
        return Point2d(x=self.x / n, y=self.y / n)

    def normalize_inplace(self):
        n = self.norm()
        if n > 0:
            self.x /= n
            self.y /= n

    def __repr__(self) -> str:
        return f"<x: {self.x}, y: {self.y}>"

    def copy(self) -> Point2d:
        return Point2d(x=self.x, y=self.y)

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"

    def round(self) -> Point2d:
        return Point2d(x=round(self.x), y=round(self.y))
    
    
PointLike = Union[Point2d, Array]



def get_mouse_pos() -> Point2d:
    mouse_pos = pygame.mouse.get_pos()
    return Point2d.make(mouse_pos)


@dataclass
class FollowMouse:
    active: bool = False
    origin_pos: Point2d = Point2d()
    origin_mouse_pos: Point2d = Point2d()

    def activate(self, pos: Point2d):
        self.active = True
        self.origin_pos = pos
        self.origin_mouse_pos = get_mouse_pos()

    def deactivate(self):
        self.active = False

    def get_updated_pos(self) -> Point2d:
        return get_mouse_pos() - self.origin_mouse_pos + self.origin_pos
    

# class Widget:
#     def __init__(self, pos: PointLike, size: PointLike):
#         self.pos = Point2d.make(pos)
#         self.size = Point2d.make(size)
#         self.follow_mouse = FollowMouse()
#         self.active = False
    
#     def mouse_in_hitbox(self) -> bool:
#         return self.pos <= get_mouse_pos() <= self.pos + self.size
    
#     def update(self):
#         if self.follow_mouse.active:
#             self.pos = self.follow_mouse.get_updated_pos()
#             return
#         if self.mouse_in_hitbox():
#             self.active = True
            

# class BTFactory:
#     def __init__(self):
#         pass













#!/usr/bin/env python3

from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from custom_msg.msg import Task
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8
from sensor_msgs.msg import JointState

import keyboard
import threading
from typing import Any, Union, List
import math
import kwak
from kwak.fullscreen import *
from kwak.interface import *
import pygame
# from kwak.fullscreen import FullScreenWidget


#pygame.init()
BIGARM = pygame.image.load(kwak_path("images/bigarm.png"))
pygame.display.set_icon(BIGARM)
pygame.display.set_caption("HDDDD")
window = pygame.display.set_mode((500, 250), pygame.RESIZABLE)
kwak.init(window, init_pygame=True)
bg_color = (0, 35, 35), (0, 35, 35), (0, 115, 60)
text_color = (255, 255, 255), (255, 255, 255), (255, 255, 255)
outline_color = (200, 200, 255), (100, 255, 100), (100, 255, 100)
p = Palette(text_color, bg_color, outline_color)
Interface.get_instance().set_global_palette(p)
Interface.get_instance().set_fps_visible(True)


# class MoveableButton(RectButton):
    
class LabelButton(RectButton):
    def __init__(self, text: str, font: pygame.font.Font):
        super().__init__(text=text, font=font)
        self.connect("clicked", self.on_click)
    
    def on_click(self):
        pass


class Canvas(FreeLayout):
    def __init__(self):
        super().__init__()


class BTFactory(FullScreenWidget):
    def __init__(self):
        super().__init__()
        self._create_app()
    
    def _create_app(self):
        font = pygame.font.SysFont("arial", 15)
        layout = HLayout()
        blocks = WidgetWithLayout()
        blocks_layout = VLayout()
        blocks_layout.add_rows([
            LabelButton(f"Button{i}", font=font)
            for i in range(200)
        ])
        blocks.set_layout(blocks_layout, Widget.TRUE_Y_SIZE)
        layout.add_row(blocks)
        canvas = Canvas()
        canvas.add_item(RectButton(text="Hello", font=font), 10, 25, hotspot=(0, 0))
        layout.add_row(canvas)
        self.set_layout(layout)
        Interface.get_instance().add_widget(self)



if __name__ == '__main__':
    BTFactory()
    Interface.get_instance().run(hz=100)
