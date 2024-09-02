from __future__ import annotations
from math import sqrt, sin, cos, asin, acos, pi
import geometry_msgs.msg as gmsg
from typing import Any, Union, Tuple, List
import random


Scalar = Union[float, int]      # TODO: add support for numpy scalar types (and potentially other scalar types as well)
Array = Union[Tuple[Scalar, ...], List[Scalar]]
array_types_checkable = (tuple, list)   # for instance checking since subscripted generics such as List[Scalar] cannot be used for instance checking
PointLike = Union[gmsg.Point, gmsg.Quaternion, Array]
QuaternionLike = Union[PointLike, Scalar]
PoseLike = Union[gmsg.Pose, Tuple[PointLike, QuaternionLike]]


def is_point_like(x: Any) -> bool:
    """
    Verify if x is convertible into Point
    """
    if isinstance(x, (gmsg.Point, gmsg.Quaternion)):
        return True
    if not isinstance(x, array_types_checkable):
        return False
    return 3 <= len(x) <= 4 and all(isinstance(v, Scalar) for v in x)


def is_quaternion_like(x: Any) -> bool:
    """
    Verify if x is convertible into Quaternion
    """
    if isinstance(x, (Scalar, gmsg.Point, gmsg.Quaternion)):
        return True
    if not isinstance(x, array_types_checkable):
        return False
    return 3 <= len(x) <= 4 and all(isinstance(v, Scalar) for v in x)


def is_pose_like(x: Any) -> bool:
    """
    Verify if x is convertible into Pose
    """
    if isinstance(x, gmsg.Pose):
        return True
    if not isinstance(x, (tuple, list)):
        return False
    if isinstance(x, list):
        print("Warning: lists are supposed to be a heterogenous type, you should use a tuple instead")
    return len(x) == 2 and is_point_like(x[0]) and is_quaternion_like(x[1])


def random_in_range(lower: float, upper: float):
    if lower == upper:
        return lower
    return lower + random.random() * (upper-lower)


class Point(gmsg.Point):    
    @classmethod
    def make(cls, value: PointLike) -> Point:
        """
        Convert a PointLike to Point
        """
        if isinstance(value, cls):
            return value
        if isinstance(value, (gmsg.Point, gmsg.Quaternion)):
            return cls(x=value.x, y=value.y, z=value.z)
        if isinstance(value, array_types_checkable):
            return cls(x=float(value[0]), y=float(value[1]), z=float(value[2]))
        raise TypeError(f"Unsuported type '{type(value).__name__}' for conversion to Point")
    
    @classmethod
    def random(cls, x_range: Tuple[float, float]=(-10.0, 10.0), y_range: Tuple[float, float]=(-10.0, 10.0), z_range: Tuple[float, float]=(-10.0, 10.0)) -> Point:
        """
        Generate a random Point in the specified ranges
        """
        return cls(
            x = random_in_range(*x_range),
            y = random_in_range(*y_range),
            z = random_in_range(*z_range)
        )
    
    def copy(self) -> Point:
        return Point(x=self.x, y=self.y, z=self.z)
    
    @classmethod
    def eq(cls, p1: PointLike, p2: PointLike, precision: float=10**-10) -> bool:
        """
        Check whether the two points are equal within some margin of error 
        (the precision margin is on the squared norm)
        """
        return (cls.make(p1) - cls.make(p2)).sq_norm() <= precision

    def xyz(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.z
    
    def __add__(self, other: PointLike) -> Point:
        # TODO: maybe add more direct typechecking via is_point_like (for now type checking is done indirectly in the make method), and same for __sub__ and __rsub__ methods
        other = Point.make(other)
        return Point(x=self.x+other.x, y=self.y+other.y, z=self.z+other.z)
    
    def __radd__(self, other: PointLike) -> Point:
        return self + other

    def __sub__(self, other: PointLike) -> Point:
        other = Point.make(other)
        return Point(x=self.x-other.x, y=self.y-other.y, z=self.z-other.z)
    
    def __rsub__(self, other: PointLike) -> Point:
        other = Point.make(other)
        return Point(x=other.x-self.x, y=other.y-self.y, z=other.z-self.z)
    
    def __mul__(self, other: Union[Scalar, PointLike]) -> Point:
        if isinstance(other, Scalar):
            return Point(x=other*self.x, y=other*self.y, z=other*self.z)
        if is_point_like(other):
            return self.dot(other)
        raise TypeError(f"Unsuported operand types for *: '{Point.__name__}' and '{type(other).__name__}'")
    
    def __rmul__(self, other: Union[Scalar, PointLike]) -> Point:
        try:
            return self * other
        except TypeError:
            raise TypeError(f"Unsuported operand types for *: '{type(other).__name__}' and '{Point.__name__}'")
    
    def __neg__(self) -> Point:
        return Point(x=-self.x, y=-self.y, z=-self.z)
    
    def dot(self, other: PointLike) -> float:
        other = Point.make(other)
        return self.x*other.x + self.y*other.y + self.z*other.z
    
    def sq_norm(self) -> float:
        return self.x**2 + self.y**2 + self.z**2
    
    def norm(self) -> float:
        return sqrt(self.sq_norm())
    
    def __abs__(self) -> float:
        return self.norm()
    
    def normalized(self) -> Point:
        n = self.norm()
        if n == 0:
            return Point(x=0.0, y=0.0, z=0.0)
        return Point(x=self.x/n, y=self.y/n, z=self.z/n)
    
    def normalize_inplace(self):
        n = self.norm()
        if n > 0:
            self.x /= n
            self.y /= n
            self.z /= n
    
    def publishable(self) -> gmsg.Point:
        return gmsg.Point(x=self.x, y=self.y, z=self.z)


class Quaternion(gmsg.Quaternion):
    @classmethod
    def make(cls, value: QuaternionLike) -> Quaternion:
        """
        Convert a QuaternionLike to Quaternion
        """
        if isinstance(value, cls):
            return value
        if isinstance(value, gmsg.Quaternion):
            return cls(w=value.w, x=value.x, y=value.y, z=value.z)
        if isinstance(value, gmsg.Point):
            return cls(w=0.0, x=value.x, y=value.y, z=value.z)
        if isinstance(value, array_types_checkable):
            return cls(w=(value[3] if len(value) >= 4 else 0.0), x=value[0], y=value[1], z=value[2])
        if isinstance(value, Scalar):
            return cls(w=float(value), x=0.0, y=0.0, z=0.0)
        raise TypeError(f"Unsuported type '{type(value).__name__}' for conversion to Quaternion")

    @classmethod
    def from_axis_angle(cls, axis: PointLike, angle: float) -> Quaternion:
        """
        Return the quaternion associated to a rotation of a certain angle around a certain axis
        """
        axis = Point.make(axis).normalized()
        x, y, z = axis.xyz()
        q = Quaternion()
        q.w = cos(angle/2)
        q.x = x*sin(angle/2)
        q.y = y*sin(angle/2)
        q.z = z*sin(angle/2)
        return q
    
    @classmethod
    def random(cls) -> Quaternion:
        """
        Generate a random normalized quaternion
        """
        x = random_in_range(-1.0, 1.0)
        norm_limit = sqrt(1-x**2)
        y = random_in_range(-norm_limit, norm_limit)
        norm_limit = sqrt(1-x**2-y**2)
        z = random_in_range(-norm_limit, norm_limit)
        w = sqrt(1-x**2-y**2-z**2)
        if random.random() < 0.5:
            w = -w
        return cls(x=x, y=y, z=z, w=w)
    
    def copy(self) -> Quaternion:
        return Quaternion(x=self.x, y=self.y, z=self.z, w=self.w)
    
    @classmethod
    def eq(cls, q1: QuaternionLike, q2: QuaternionLike, precision: float=10**-10) -> bool:
        """
        Check whether the two quaternions are equal within some margin of error 
        (the precision margin is on the squared norm)
        """
        return (cls.make(q1) - cls.make(q2)).sq_norm() <= precision

    @staticmethod
    def reverse_trig(cost: float, sint: float) -> float:
        """
        Retrieve angle from its cos and sin
        """
        if cost >= 0 and sint >= 0:
            angle = asin(sint)
        elif cost >= 0 and sint <= 0:
            angle = asin(sint)
        elif cost <= 0 and sint >= 0:
            angle = acos(cost)
        else:
            angle = -acos(cost)
        return angle
    
    def wxyz(self) -> Tuple[float, float, float, float]:
        return self.w, self.x, self.y, self.z
    
    def xyzw(self) -> Tuple[float, float, float, float]:
        return self.x, self.y, self.z, self.w

    def xyz(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.z
    
    def __add__(self, other: QuaternionLike) -> Quaternion:
        # TODO: maybe add more direct typechecking via is_point_like (for now type checking is done semi directly in the make method), and same for __sub__ and __rsub__ methods
        other = Quaternion.make(other)
        return Quaternion(w=self.w+other.w, x=self.x+other.x, y=self.y+other.y, z=self.z+other.z)

    def __radd__(self, other: QuaternionLike) -> Quaternion:
        return self + other
    
    def __sub__(self, other: QuaternionLike) -> Quaternion:
        other = Quaternion.make(other)
        return Quaternion(w=self.w-other.w, x=self.x-other.x, y=self.y-other.y, z=self.z-other.z)
    
    def __rsub__(self, other: QuaternionLike) -> Quaternion:
        other = Quaternion.make(other)
        return Quaternion(w=other.w-self.w, x=other.x-self.x, y=other.y-self.y, z=other.z-self.z)
    
    @staticmethod
    def _quat_mul(q1: gmsg.Quaternion, q2: gmsg.Quaternion) -> Quaternion:
        return Quaternion(
            w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
            x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
            y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
            z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
        )
    
    def _scalar_mul(self, scalar: Scalar) -> Quaternion:
        if not isinstance(scalar, Scalar):
            raise TypeError     # TODO: write error message
        return Quaternion(w=scalar*self.w, x=scalar*self.x, y=scalar*self.y, z=scalar*self.z)
    
    def __mul__(self, other: QuaternionLike) -> Quaternion:
        if isinstance(other, Scalar):
            self._scalar_mul(other)
        if is_quaternion_like(other):
            other = Quaternion.make(other)
            return self._quat_mul(self, other)
        raise TypeError(f"Unsuported operand types for *: '{type(other).__name__}' and '{Quaternion.__name__}'")
    
    def __rmul__(self, other: QuaternionLike) -> Quaternion:
        if isinstance(other, Scalar):
            self._scalar_mul(other)
        if is_quaternion_like(other):
            other = Quaternion.make(other)
            return self._quat_mul(other, self)
        raise TypeError(f"Unsuported operand types for *: '{Quaternion.__name__}' and '{type(other).__name__}'")
    
    def __neg__(self) -> Quaternion:
        return Quaternion(w=-self.w, x=-self.x, y=-self.y, z=-self.z)

    def sq_norm(self) -> float:
        return self.w**2 + self.x**2 + self.y**2 + self.z**2
    
    def norm(self) -> float:
        return sqrt(self.sq_norm())
    
    def __abs__(self) -> float:
        return self.norm()

    def inv(self, suppose_unit_norm=True) -> Quaternion:
        """
        Quaternion multiplicative inverse
        """
        n = 1.0 if suppose_unit_norm else 1/self.norm()
        q_ = Quaternion(
            w = self.w * n,
            x = -self.x * n,
            y = -self.y * n,
            z = -self.z * n
        )
        return q_
    
    def normalized(self) -> Quaternion:
        n = self.norm()
        if n == 0:
            return Quaternion(w=0.0, x=0.0, y=0.0, z=0.0)
        return Quaternion(w=self.w/n, x=self.x/n, y=self.y/n, z=self.z/n)
    
    def normalize_inplace(self):
        n = self.norm()
        if n > 0:
            self.w /= n
            self.x /= n
            self.y /= n
            self.z /= n
    
    def reverse_quat(self) -> Tuple[Point, float]:
        """
        Retrieve axis and angle of rotation from quaternion
        """
        cos_t = self.w
        sin_t = Point.make(self).norm()
        if sin_t != 0:
            axis = Point(x=self.x/sin_t, y=self.y/sin_t, z=self.z/sin_t)
        else:
            axis = Point(x=0.0, y=0.0, z=0.0)
        angle = 2*Quaternion.reverse_trig(cos_t, sin_t)
        return axis, angle
    
    def point_image(self, point: PointLike) -> Point:
        """
        Calculate the image of a point under the rotation described by the quaternion
        """
        p = Quaternion.make(Point.make(point))
        self_inv = self.inv()
        p = self * p * self_inv
        return Point.make(p)
    
    def turn_around(self, axis: PointLike=(1.0, 0.0, 0.0), angle: float=pi) -> Quaternion:
        """
        Calculate the quaternion corresponding to the rotation described by self composed with the rotation described by the given axis and angle
        """
        axis = self.point_image(axis)
        r = Quaternion.from_axis_angle(axis, angle)
        return r * self
    
    def publishable(self) -> gmsg.Quaternion:
        return gmsg.Quaternion(x=self.x, y=self.y, z=self.z, w=self.w)


class Pose(gmsg.Pose):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.position: Quaternion = Point.make(self.position)
        self.orientation: Quaternion = Quaternion.make(self.orientation)
    
    @classmethod
    def make(cls, value: PoseLike, normalize=False) -> Pose:
        """
        Convert a PoseLike to Pose
        :param normalize: whether to normalize the given quaternion 
        (note that for a correct rotation description the quaternion must be normalized so this should be set to False only if an already normalized quaternion is passed)
        """
        if isinstance(value, gmsg.Pose):
            orientation = Quaternion.normalized(value.orientation) if normalize else value.orientation
            return cls(position=value.position, orientation=orientation)
        if isinstance(value, (list, tuple)):
            position, orientation = value
            position = Point.make(position)
            orientation = Quaternion.make(orientation)
            if normalize:
                orientation.normalize_inplace()
            return cls(position=position, orientation=orientation)
        raise TypeError(f"Unsuported type '{type(value).__name__}' for conversion to Pose")
    
    @classmethod
    def random(cls, x_range: Tuple[float, float]=(-10.0, 10.0), y_range: Tuple[float, float]=(-10.0, 10.0), z_range: Tuple[float, float]=(-10.0, 10.0)) -> Pose:
        """
        Generate a random Pose with position in the specified ranges
        """
        return Pose(
            position = Point.random(x_range, y_range, z_range),
            orientation = Quaternion.random()
        )
    
    def copy(self) -> Pose:
        return Pose(position=self.position.copy(), orientation=self.orientation.copy())
    
    @classmethod
    def eq(cls, pose1: PoseLike, pose2: PoseLike, precision: float=10**-10) -> bool:
        """
        Check whether the two poses are equal within some margin of error 
        (the precision margin is on the squared norm of the position and orientation).
        Both position and orientation need to be within the requested precision margin.
        """
        pose1, pose2 = cls.make(pose1), cls.make(pose2)
        return Point.eq(pose1.position, pose2.position, precision) and Quaternion.eq(pose1.orientation, pose2.orientation, precision)
    
    def point_image(self, point: PointLike) -> Point:
        return self.position + self.orientation.point_image(point)

    def inv(self) -> Pose:
        """
        Return inv_pose such that the composition of self and inv_pose is the trivial pose
        """
        orientation = self.orientation.inv()
        position = -orientation.point_image(self.position)
        return Pose(position=position, orientation=orientation)
    
    def compose(self, other: PoseLike) -> Pose:
        """
        Compose the transformations corresponding to self and the other pose in the following way:
        self with respect to origin, other with respect to self
        """
        if not is_pose_like(other):
            raise TypeError(f"Cannot compose Pose with instance of non PoseLike type {type(other).__name__}")
        other = Pose.make(other)
        position = self.point_image(other.position)
        # The required orientation is the rotation corresponding to the composition of the rotation of the first pose (self) 
        # and the rotation of the second pose (other) but in the frame obtained after applying the first rotation.
        # This is NOT equivalent to the simple composition of the first and second rotation,
        # however it turns out it is equivalent to the simple composition in the reverse order : second then first rotation 
        # (recall quaternion multiplication is non commutative).
        # Due to the way quaternions are used to apply rotations, this corresponds to the following multiplication.
        orientation = self.orientation * other.orientation
        return Pose(position=position, orientation=orientation)
    
    def compose_inplace(self, other: PoseLike):
        """
        Same as the compose method but in place
        """
        if not is_pose_like(other):
            raise TypeError(f"Cannot compose Pose with instance of non PoseLike type {type(other).__name__}")
        self.position = self.point_image(other.position)
        self.orientation *= other.orientation
    
    def __matmul__(self, other: PoseLike) -> Pose:
        return self.compose(other)
    
    def __rmatmul__(self, other: PoseLike) -> Pose:
        other = Pose.make(other)
        return other.compose(self)
    
    def compose_multiple(self, *poses: PoseLike) -> Pose:
        """
        Compose mutliple poses, can also be called without an instance 
        in the (kinda unclean but who cares, it's python) form Pose.compose_multiple(pose1, ..., pose_k)
        where pose1, ..., pose_k are pose-like
        """
        res = Pose.make(self)   # this is done in case the method is called without an instance
        for pose in poses:
            res.compose_inplace(pose)
        return res

    def publishable(self) -> gmsg.Pose:
        return gmsg.Pose(position=self.position.publishable(), orientation=self.orientation.publishable())


def publishable(obj: Union[gmsg.Point, gmsg.Quaternion, gmsg.Pose]):
    if isinstance(obj, gmsg.Point):
        return Point.make(obj).publishable()
    if isinstance(obj, gmsg.Quaternion):
        return Quaternion.make(obj).publishable()
    if isinstance(obj, gmsg.Pose):
        return Pose.make(obj).publishable()
    raise TypeError