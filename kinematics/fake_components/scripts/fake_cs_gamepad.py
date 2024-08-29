#!/usr/bin/env python3

from typing import Any
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import evdev
import evdev.events
import threading
from time import sleep
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int8, Bool
from custom_msg.msg import Task
import math
import itertools
from collections.abc import Callable
from typing import List, Dict, Any, Union


def clean(x: float) -> float:
    epsilon = 0.09
    return 0 if abs(x) < epsilon else x


def str_pad(x: Any, length=10) -> str:
    s = str(x)
    if len(s) >= length:
        return s[:length]
    return s + " " * (length - len(s))


def add(l1: list, l2: list) -> list:
    return [x + y for x, y in zip(l1, l2)]


def normalized(l: list) -> list:
    n = math.sqrt(sum(x**2 for x in l))
    if n == 0:
        return l
    return [x / n for x in l]


class EventFuncWrapper:
    """
    keeping information about a function to be called upon a certain event, the function must take the event value as a parameter
    """

    def __init__(
        self, func: Callable = None, event_value_arg_name="event_value", **kwargs
    ):
        """
        :param func: the function to call
        :param event_value_arg_name: the name of the function's parameter for the event value
        :param kwargs: additional arguments of the function
        """
        if func is None:
            func = lambda *args, **kwargs: None
        self.func = func
        self.event_value_arg_name = event_value_arg_name
        self.kwargs = kwargs
        self.kwargs[event_value_arg_name] = None

    def __call__(self, event_value) -> Any:
        self.kwargs[self.event_value_arg_name] = event_value
        return self.func(**self.kwargs)


class GamePadConfig:
    """
    Full gamepad configuration comprising input ids, value offsets and amplitudes.
    Also allows to bind functions to be called upon the event of a certain input of the gamepad.
    """

    # TODO: create an interface for the touchpad
    NULL = -1
    TRIANGLE = 0
    SQUARE = 1
    CIRCLE = 2
    CROSS = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    L3 = 8
    R3 = 9
    CREATE = 10
    OPTIONS = 11
    PS = 12
    TOUCHPAD = 13
    LH = 14
    LV = 15
    RH = 16
    RV = 17
    DIRH = 18
    DIRV = 19

    BUTTONS = {
        TRIANGLE: "triangle",
        SQUARE: "square",
        CIRCLE: "circle",
        CROSS: "cross",
        L1: "L1",
        R1: "R1",
        L3: "L3",
        R3: "R3",
        CREATE: "create",
        OPTIONS: "options",
        PS: "PS",
        TOUCHPAD: "touchpad",
    }
    ANALOG_TRIGGERS = {
        RH: "RH",
        RV: "RV",
        LH: "LH",
        LV: "LV",
        L2: "L2",
        R2: "R2",
        DIRH: "dirH",
        DIRV: "dirV",
    }
    INPUTS = BUTTONS | ANALOG_TRIGGERS
    DEFAULT_IDS = {inp: -1 for inp in INPUTS}
    DEFAULT_IDS |= {
        L1: 310,
        R1: 311,
        L2: 2,
        R2: 5,
        PS: 316,
        LH: 0,
        LV: 1,
        RH: 3,
        RV: 4,
        DIRH: 16,
        DIRV: 17,
        TRIANGLE: 307,
        SQUARE: 308,
        CIRCLE: 305,
        CROSS: 304,
    }

    def __init__(self, **kwargs):
        """
        :param kwargs: Must contain all the ids, offsets and amplitudes of the inputs that need to be set to a different than default value.
            All parameters need to be given as keyword arguments in the format [name of input]_id, [name of input]_offset and [name of input]_amplitude.
            Default ids are given by GamePadConfig.DEFAULT_IDS, default offsets are set to 0,  default amplitudes are set to 1.
        """
        self.input_ids: Dict[int, int] = {}
        self.input_offsets: Dict[int, float] = {}
        self.input_amplitudes: Dict[int, float] = {}
        for input, name in GamePadConfig.INPUTS.items():
            attr = f"{name}_id"
            self.input_ids[input] = kwargs.get(attr, GamePadConfig.DEFAULT_IDS[input])
            attr = f"{name}_offset"
            self.input_offsets[input] = kwargs.get(attr, 0.0)
            attr = f"{name}_amplitude"
            self.input_amplitudes[input] = kwargs.get(attr, 1.0)

        # dict of inputs correspondig to ids (if multiple inputs have the same id, for instance -1, the behaviour is undefined)
        self.reverse_id_dict: Dict[int, int] = {}
        for inp, id in self.input_ids.items():
            self.reverse_id_dict[id] = inp
        self.fast_id_finding = True

        # list of functions to be called on event for every input of the gamepad
        self.event_signals: Dict[int, List[Callable]] = {
            k: [] for k in GamePadConfig.INPUTS
        }

    def bind(
        self, input: int, func: Callable, event_value_arg_name: str, **kwargs
    ) -> None:
        """
        Bind a function call to the trigger event of a certain gamepad input.
        :param input: a gamepad input type
        :param func: the function to bind (it needs to take the event value as a parameter)
        :param event_value_arg_name: the name of the function's parameter for the event value
        :param kwargs: other arguments of the function given as keyword arguments
        """
        if input in GamePadConfig.INPUTS:
            self.event_signals[input].append(
                EventFuncWrapper(func, event_value_arg_name, **kwargs)
            )

    def find_input(self, id) -> int:
        """
        Determine the gamepad input that has the given id.
        :return: a gamepad input type
        """
        if self.fast_id_finding:
            return self.reverse_id_dict.get(id, GamePadConfig.NULL)
        for input, inp_id in self.input_ids.items():
            if inp_id == id:
                return input
        return GamePadConfig.NULL

    def handle_event(self, event: evdev.events.InputEvent) -> None:
        """
        Responsible for calling approriate binded functions corresponding to event
        """
        if event.type == 0:
            return  # ignore this type of event (not sure what it corresponds to but results in weird behaviour)
        input = self.find_input(event.code)
        if input == GamePadConfig.NULL:
            return
        self.emmit_signal(input, event.value)

    def emmit_signal(self, input: int, raw_value: float) -> None:
        """
        Call functions binded to input.
        :param input: a gamepad input type
        :param raw_value: raw value of the event, to be transformed to the desired range using the corresponding offset and amplitude of the config
        """
        offset = self.input_offsets[input]
        amplitude = self.input_amplitudes[input]
        event_value = (raw_value - offset) / amplitude
        for func in self.event_signals[input]:
            func(event_value)

    @classmethod
    def from_name(cls, name: str):
        """
        Construct a GamePadConfig from the name of a known device (returns default config if the name is not known)
        """
        if name == "Generic X-Box pad":
            return cls(
                circle_id=305,
                triangle_id=308,
                square_id=307,
                cross_id=304,
                RH_amplitude=2**15,
                RV_amplitude=2**15,
                LH_amplitude=2**15,
                LV_amplitude=2**15,
                R2_amplitude=2**8,
                L2_amplitude=2**8,
            )
        elif name == "Wireless Controller":
            return cls(
                circle_id=305,
                triangle_id=307,
                square_id=308,
                cross_id=304,
                RH_offset=2**7,
                RV_offset=2**7,
                LH_offset=2**7,
                LV_offset=2**7,
                RH_amplitude=2**7,
                RV_amplitude=2**7,
                LH_amplitude=2**7,
                LV_amplitude=2**7,
                R2_amplitude=2**8,
                L2_amplitude=2**8,
            )
        return cls()


class EnumOld:
    # kinda overkill but for fun to mimic a C-style enum + some extra functionalities

    def __init__(self, **kwargs):
        self.items = kwargs
        self.slots = list(kwargs)
        self.slot_values = [kwargs[slot] for slot in self.slots]
        # print(self.slots)
        for k in self.slots:
            setattr(self, k, kwargs[k])

    def __iter__(self):
        for k in self.slots:
            yield self.items[k]

    def __len__(self):
        return len(self.slots)

    def __getitem__(self, index):
        k = self.slots[index % len(self.slots)]
        return self.items[k]

    def next(self, val):
        i = (self.slot_values.index(val) + 1) % len(self.slots)
        return self.slot_values[i]


def Enum(**kwargs):
    """
    *** very overkill and very useless but I was bored ***
    Tries to mimic a C-style enum with some additional useful properties.
    :param kwargs: the members of the enum
    :return: A class type having as class attributes instances of that class corresponding to the members of the enum.
        Thanks to the class being constructed on a custom metaclass, iteration, len computation and item query can be performed directly on the enum class object.
    """
    if len(kwargs) == 0:
        ValType = int  # by default
    else:
        ValType = type(list(kwargs.values())[0])

    class EnumMetaClass(type):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.__SLOTS = []

        def __iter__(self):
            for obj in self.__SLOTS:
                yield obj

        def __len__(self):
            return len(self.__SLOTS)

        def __getitem__(self, index: int):
            obj = self.__SLOTS[index % len(self.__SLOTS)]
            return obj

        def next(self, obj):
            for i, slot in enumerate(self.__SLOTS):
                if slot == obj:
                    break
            return self.__SLOTS[(i + 1) % len(self.__SLOTS)]

        def register_instance(self, name: str, value: ValType):
            setattr(self, name, value)
            self.__SLOTS.append(getattr(self, name))

    class EnumClassTemplate(metaclass=EnumMetaClass):
        def __init__(self, name: str, value: ValType):
            self.name = name
            self.value = value
            type(self).register_instance(name, self)

        def __eq__(self, other: Any):
            if isinstance(other, type(self)):
                return other.value == self.value
            elif isinstance(other, type(self.value)):
                return other == self.value
            return False

        def next(self):
            return type(type(self)).next(type(self), self)

        def __repr__(self):
            return f"<name: {self.name}, value: {self.value}>"

    EnumClass = EnumMetaClass(
        "EnumClass",
        (object,),
        {
            "__init__": EnumClassTemplate.__init__,
            "__eq__": EnumClassTemplate.__eq__,
            "__repr__": EnumClassTemplate.__repr__,
            "next": EnumClassTemplate.next,
        },
    )
    for name, value in kwargs.items():
        EnumClass(name, value)
    EnumClass.__new__ = (
        lambda *args, **kwargs: None
    )  # class won't be instantiable anymore
    return EnumClass


HDMode = EnumOld(
    IDLE=-1,
    MANUAL_DIRECT=1,
    SEMI_AUTONOMOUS=2,
    MANUAL_INVERSE=0,
    # AUTONOMOUS = 3
)


SemiAutoTask = EnumOld(
    NO_TASK=Task.NO_TASK,
    BTN_TASK=Task.BUTTON,
    PLUG_VOLTMETER=Task.PLUG_VOLTMETER_APPROACH,
    NAMED_TARGET_TASK=Task.NAMED_TARGET,
)


class ControlStation(Node):
    """
    Class reading gamepad and sending commands (to handling device) accordingly
    """

    def __init__(self):
        super().__init__("fake_cs_gamepad")

        self.vel_cmd = [0.0] * 8
        self.axis_cmd = [0.0] * 3
        self.man_inv_axis = [0.0] * 3
        self.man_inv_twist = Twist()
        self.man_inv_velocity_scaling = 1.0
        self.semi_auto_cmd = Task.NO_TASK

        # direction of joint 3, 4
        self.joint3_dir = 1
        self.joint4_dir = 1

        self.hd_mode = HDMode.IDLE  # HDMode.MANUAL_DIRECT

        self.joint_vel_cmd_pub = self.create_publisher(
            Float32MultiArray, "/CS/HD_gamepad", 10
        )
        self.man_inv_axis_pub = self.create_publisher(
            Float32MultiArray, "/ROVER/HD_man_inv_axis", 10
        )
        self.man_inv_twist_pub = self.create_publisher(
            Twist, "/ROVER/HD_man_inv_twist", 10
        )
        self.task_pub = self.create_publisher(Task, "/ROVER/semi_auto_task", 10)
        self.mode_change_pub = self.create_publisher(Int8, "/ROVER/HD_mode", 10)
        self.timer_period = 1 / 30
        self.timer = None
        self.gamepad = None
        self.connect()

        self.gamepad_config = GamePadConfig()
        self.identify_device()
        self.create_bindings()

    def create_bindings(self):
        for i, input in enumerate(
            [
                GamePadConfig.RH,
                GamePadConfig.RV,
                GamePadConfig.R2,
                GamePadConfig.L2,
                GamePadConfig.LV,
                GamePadConfig.LH,
            ]
        ):
            self.gamepad_config.bind(
                input, self.set_manual_velocity, "value", joint_index=i
            )

        self.gamepad_config.bind(
            GamePadConfig.R1, self.flip_manual_velocity_dir, "value", joint_index=2
        )
        self.gamepad_config.bind(
            GamePadConfig.L1, self.flip_manual_velocity_dir, "value", joint_index=3
        )
        self.gamepad_config.bind(GamePadConfig.PS, self.switch_mode, "do")

        for val, input in zip(
            [1.0, -1.0, 0.1, -0.1],
            [
                GamePadConfig.CIRCLE,
                GamePadConfig.SQUARE,
                GamePadConfig.TRIANGLE,
                GamePadConfig.CROSS,
            ],
        ):
            self.gamepad_config.bind(
                input, self.set_gripper_speed, "event_value", value=val
            )

        self.gamepad_config.bind(
            GamePadConfig.DIRH, self.set_rassor_speed, "event_value", value=1.0
        )
        self.gamepad_config.bind(
            GamePadConfig.DIRV, self.set_rassor_speed, "event_value", value=-0.1
        )

        for i, input in enumerate(
            [
                GamePadConfig.L2,
                GamePadConfig.R2,
                GamePadConfig.CIRCLE,
                GamePadConfig.SQUARE,
                GamePadConfig.TRIANGLE,
                GamePadConfig.CROSS,
            ]
        ):
            self.gamepad_config.bind(
                input,
                self.set_man_inv_axis,
                "value",
                coordinate=i // 2,
                multiplier=(-1) ** i,
            )
            self.gamepad_config.bind(
                input, self.set_semi_auto_cmd, "event_value", index=i
            )

        for i, input in enumerate([GamePadConfig.DIRH, GamePadConfig.DIRV]):
            self.gamepad_config.bind(
                input, self.set_man_inv_angular, "value", coordinate=i // 2
            )

        self.gamepad_config.bind(
            GamePadConfig.LV, self.set_man_inv_velocity_scaling, "value"
        )

    def connect(self):
        print("connecting")
        while self.gamepad is None:
            for device in map(evdev.InputDevice, evdev.list_devices()):
                if device.name.strip() != "Generic X-Box pad":
                    continue
                print(device)
                self.gamepad = device
                # self.timer.start()
                self.timer = self.create_timer(self.timer_period, self.publish_cmd)
                return device
            sleep(1)

    def identify_device(self):
        name = self.gamepad.name
        self.gamepad_config = GamePadConfig.from_name(name)

    def publish_cmd(self):
        if self.hd_mode == HDMode.IDLE:
            return
        elif self.hd_mode == HDMode.MANUAL_DIRECT:
            l = list(map(clean, map(float, self.vel_cmd)))
            l[2] *= self.joint3_dir
            l[3] *= self.joint4_dir
            print("[", ", ".join(map(str_pad, l)), "]")
            l = [1.0] + l  # add dummy velocity scaling factor
            self.joint_vel_cmd_pub.publish(Float32MultiArray(data=l))
        elif self.hd_mode == HDMode.MANUAL_INVERSE:
            axis = normalized(self.man_inv_axis[:3])
            print("[", ", ".join(map(str_pad, axis)), "]")
            data = [self.man_inv_velocity_scaling] + axis
            self.man_inv_axis_pub.publish(Float32MultiArray(data=data))
            # print(self.man_inv_twist)
            # self.man_inv_twist_pub.publish(self.man_inv_twist)
        elif self.hd_mode == HDMode.SEMI_AUTONOMOUS:
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NO_TASK:
                return
            msg = Task(type=self.semi_auto_cmd)
            if self.semi_auto_cmd == Task.NAMED_TARGET:
                msg.str_slot = "optimal_view"
            self.task_pub.publish(msg)
            self.semi_auto_cmd = Task.NO_TASK

    def switch_mode(self, do=1):
        if not do:
            return
        # self.hd_mode = (self.hd_mode + 1) % len(HDMode)
        self.hd_mode = HDMode.next(self.hd_mode)
        msg = Int8(data=self.hd_mode)
        self.mode_change_pub.publish(msg)

    def set_manual_velocity(self, joint_index, value):
        if self.hd_mode != HDMode.MANUAL_DIRECT:
            return
        self.vel_cmd[joint_index] = value

    def flip_manual_velocity_dir(self, joint_index, value):
        if self.hd_mode != HDMode.MANUAL_DIRECT:
            return
        if joint_index == 2:
            self.joint3_dir *= -1
        elif joint_index == 3:
            self.joint4_dir *= -1

    def set_gripper_speed(self, value, event_value):
        if self.hd_mode != HDMode.MANUAL_DIRECT:
            return
        self.vel_cmd[6] = value * event_value

    def set_rassor_speed(self, value, event_value):
        if self.hd_mode != HDMode.MANUAL_DIRECT:
            return
        self.vel_cmd[7] = value * event_value

    def set_man_inv_axis(self, coordinate, value, multiplier=1):
        if self.hd_mode != HDMode.MANUAL_INVERSE:
            return
        self.man_inv_axis[coordinate] = 0.0 if value < 0.5 else 1.0 * multiplier
        return
        v = 0.0 if value < 0.5 else 1.0 * multiplier
        if coordinate == 0:
            self.man_inv_twist.linear.x = v
        elif coordinate == 1:
            self.man_inv_twist.linear.y = v
        else:
            self.man_inv_twist.linear.z = v

    def set_man_inv_angular(self, coordinate, value, multiplier=1):
        if self.hd_mode != HDMode.MANUAL_INVERSE:
            return
        v = 0.0 if value < 0.5 else 1.0 * multiplier
        if coordinate == 0:
            self.man_inv_twist.angular.x = v
        elif coordinate == 1:
            self.man_inv_twist.angular.y = v
        else:
            self.man_inv_twist.angular.z = v

    def set_semi_auto_cmd(self, index, event_value):
        if self.hd_mode != HDMode.SEMI_AUTONOMOUS:
            return
        if event_value != 1:
            return
        self.semi_auto_cmd = SemiAutoTask[index]

    def set_man_inv_velocity_scaling(self, value):
        if self.hd_mode != HDMode.MANUAL_INVERSE:
            return
        self.man_inv_velocity_scaling = 1.0 - abs(value)

    def read_gamepad(self):
        while rclpy.ok():
            try:
                for event in self.gamepad.read_loop():
                    self.gamepad_config.handle_event(event)

            except (TypeError, IOError) as e:
                raise
                self.timer.cancel()
                self.connect()
                print(e)


def main():
    rclpy.init()
    node = ControlStation()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        node.read_gamepad()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == "__main__":
    main()
