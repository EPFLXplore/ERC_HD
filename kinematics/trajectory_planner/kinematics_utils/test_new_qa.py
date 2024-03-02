import kinematics_utils.quaternion_arithmetic_old as qao
import kinematics.trajectory_planner.kinematics_utils.quaternion_arithmetic as qan
import geometry_msgs.msg as gmsg
from math import sqrt, pi
import random
from typing import Callable, Dict, Any, Tuple, Union, Type, List
import itertools


class Angle(float):
    @classmethod
    def random(cls):
        return cls(2*pi*random.random())
    

class Axis(tuple):    
    @classmethod
    def random(cls) -> Tuple[float, float, float]:
        x = qan.random_in_range(-1.0, 1.0)
        norm_limit = sqrt(1-x**2)
        y = qan.random_in_range(-norm_limit, norm_limit)
        z = sqrt(1-x**2-y**2)
        if random.random() < 0.5:
            z = -z
        return (x, y, z)


def generate_random_params(func: Callable) -> Dict[str, Any]:
    """
    Generate random parameters to be given to the function func. 
    The required type for the parameters is infered by annotations, so this will produce reliable result only for a fully annotated function.
    Currently supported parameter types are : bool, int, float, Angle, Axis, Point, Quaternion, Pose.
    Does not support functions with variable positional or variable keyword arguments (that is the syntax *args, **kwargs).

    :param func: the function which parameters will be generated for
    :return: a dict of keyword parameters
    """
    params = {}
    for arg, arg_type in func.__annotations__.items():
        if arg == "return": continue
        params[arg] = (
            random.random() < 0.5       if arg_type is bool else
            random.randint(-10, 10)     if arg_type is int else
            2*random.random() - 1       if arg_type is float else
            Angle.random()              if arg_type is Angle else
            Axis.random()               if arg_type is Axis else
            qan.Point.random()          if issubclass(arg_type, gmsg.Point) else
            qan.Quaternion.random()     if issubclass(arg_type, gmsg.Quaternion) else
            qan.Pose.random()           if issubclass(arg_type, gmsg.Pose) else
            None
        )
    return params


SingleTest = Callable[..., bool]
UnitTest = Callable[[int], list]

def unit_test(test_func: SingleTest) -> UnitTest:
    """
    Create a function that tests a given test_func on several random parameter sets.

    :param test_func: a function returning a boolean. It must be fully annotated 
    in order for random parameters of the correct type to be possible to generate via the generate_random_params function.
    :return: a function that runs test_func on an input-given number of randomly generated parameter sets 
    and gives the list of parameters for which the test failed (test_func returned False)
    """
    def test(n: int=10) -> list:
        errors = []
        for _ in range(n):
            params = generate_random_params(test_func)
            if not test_func(**params):
                errors.append(params)
        return errors
    return test


GeometryType = Union[gmsg.Point, gmsg.Quaternion, gmsg.Pose]
NewGeometryType = Union[qan.Point, qan.Quaternion, qan.Pose]

def convert(a: GeometryType) -> NewGeometryType:
    if isinstance(a, gmsg.Point):
        return qan.Point.make(a)
    if isinstance(a, gmsg.Quaternion):
        return qan.Quaternion.make(a)
    if isinstance(a, gmsg.Pose):
        return qan.Pose.make(a)
    raise TypeError

def eq(*values: GeometryType) -> bool:
    precision = 10**-10
    values = tuple(map(convert, values))
    if len(values) < 2:
        return True
    value_type: Type[NewGeometryType] = type(values[0])
    return all(value_type.eq(values[0], x, precision) for x in values)


@unit_test
def test_quat(axis: Axis, angle: Angle) -> bool:
    return eq(qao.quat(axis, angle), qan.Quaternion.from_axis_angle(axis, angle))


@unit_test
def test_turn_around(q: qan.Quaternion, axis: Axis, angle: Angle) -> bool:
    return eq(qao.turn_around(q, axis, angle), q.turn_around(axis, angle))


@unit_test
def test_mul(q1: qan.Quaternion, q2: qan.Quaternion) -> bool:
    return eq(qao.mul(q1, q2), q1*q2)


@unit_test
def test_point_image(point: qan.Point, q: qan.Quaternion) -> bool:
    return eq(qao.point_image(point, q), q.point_image(point))


@unit_test
def test_point_object_image(point: qan.Point, pose: qan.Pose) -> bool:
    return eq(qao.point_object_image(point, pose), pose.point_image(point))


@unit_test
def test_reverse_pose(pose: qan.Pose) -> bool:
    return eq(qao.reverse_pose(pose), pose.inv())


@unit_test
def test_compose_poses(pose1: qan.Pose, pose2: qan.Pose) -> bool:
    return eq(qao.compose_poses(pose1, pose2), pose1.compose(pose2))


@unit_test
def test_compose_multiple_poses(pose1: qan.Pose, pose2: qan.Pose, pose3: qan.Pose) -> bool:
    return eq(
        qao.compose_multiple_poses(pose1, pose2, pose3), 
        pose1.compose_multiple(pose2, pose3), 
        qan.Pose.compose_multiple(pose1, pose2, pose3),
    )


tests: List[UnitTest] = [
    test_quat, 
    test_turn_around, 
    test_mul, 
    test_point_image, 
    test_point_object_image, 
    test_reverse_pose, 
    test_compose_poses, 
    test_compose_multiple_poses,
]


def validate(test: UnitTest, n=100):
    return len(test(n)) == 0



def instance_checker(arg: Any, arg_type: Type, *, complete: bool=False, max_depth: int=-1, max_breadth: int=-1) -> bool:
    # for now python 3.10 is required for this function (because of match statement syntax)
    if max_depth == 0:
        return True
    max_depth = -1 if max_depth == -1 else max_depth-1
    _slice = lambda it: itertools.islice(it, None if max_breadth == -1 else max_breadth)
    import sys
    if sys.version_info.minor < 10:
        return isinstance(arg, arg_type)

    try:
        return isinstance(arg, arg_type)
    except TypeError:
        pass

    match arg_type.__name__:
        case "List" | "list":
            elem_type = arg_type.__args__[0]
            if not isinstance(arg, list):
                return False
            if len(arg) == 0:
                return True
            if complete:
                return all(instance_checker(elem, elem_type, complete=True, max_depth=max_depth, max_breadth=max_breadth)
                           for elem in _slice(arg))
            else:
                return instance_checker(arg[0], elem_type, max_depth=max_depth, max_breadth=max_breadth)
        case "Tuple" | "tuple":
            for i, elem_type in enumerate(_slice(arg_type.__args__)):
                if not instance_checker(arg[i], )
    return isinstance(arg, arg_type)


def fully_annotated(func: Callable) -> bool:
    return True


def special_args(func: Callable) -> Tuple[str, str]:
    # bug: doesn't work if **kwargs is present but not *args
    argnames = set(func.__code__.co_varnames[:func.__code__.co_argcount])
    annot: Dict[str, Type] = func.__annotations__
    has_var_pos_args = False
    var_pos_args_name = ""
    has_var_kw_args = False
    var_kw_args_name = ""
    for arg in annot:
        if arg != "return" and arg not in argnames:
            if not has_var_pos_args:
                has_var_pos_args = True
                var_pos_args_name = arg
            elif not has_var_kw_args:
                has_var_kw_args = True
                var_kw_args_name = arg
            else:
                raise KeyError("wtf this shouldn't happen")
    return var_pos_args_name, var_kw_args_name


def type_check(func: Callable) -> Callable:
    if not fully_annotated(func):
        raise ValueError(f"only a fully annotated function can be converted into a type checked function")
    argnames = func.__code__.co_varnames[:func.__code__.co_argcount]
    type_hints: Dict[str, Type] = func.__annotations__
    var_pos_args_name, var_kw_args_name = special_args(func)

    def type_checked(*args, **kwargs):
        def _raise(arg, value): raise TypeError(f"function '{func.__name__}' expected type '{type_hints[arg].__name__}', not '{type(value).__name__}', for argument '{arg}'")
        if len(args) > len(argnames) and var_pos_args_name == "":
            raise TypeError("wrong number of positional arguments")
        for arg, value in zip(argnames, args):
            if not instance_checker(value, type_hints[arg]):
                _raise(arg, value)
        for value in args[len(argnames):]:
            if not instance_checker(value, type_hints[var_pos_args_name]):
                raise TypeError(f"function '{func.__name__}' expected type '{type_hints[var_pos_args_name].__name__}', not '{type(value).__name__}', for additional positional arguments")
        for arg, value in kwargs.items():
            if arg in argnames:
                if not instance_checker(value, type_hints[arg]):
                    _raise(arg, value)
            else:
                if var_kw_args_name == "":
                    raise TypeError("unexpected keyword argument")
                if not instance_checker(value, type_hints[var_kw_args_name]):
                    raise TypeError(f"function '{func.__name__}' expected type '{type_hints[var_kw_args_name].__name__}', not '{type(value).__name__}', for additional keyword arguments")
        return func(*args, **kwargs)
    
    type_checked.__annotations__ = func.__annotations__
    return type_checked
