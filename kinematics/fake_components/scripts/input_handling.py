from __future__ import annotations
from typing import Any
import evdev
import evdev.events
import threading
from time import sleep
import math
import itertools
import copy
from dataclasses import dataclass
from collections.abc import Callable
from typing import List, Dict, Tuple, Any, Union
import keyboard
import keyboard._keyboard_event
keyboard._os_keyboard.init()

OS_KEYBOARD_INPUTS = copy.deepcopy(keyboard._os_keyboard.from_name)
OS_KEYBOARD_INPUTS: Dict[str, List[Tuple[int, Tuple[str, ...]]]]
for inp in ("A", "D", "M", "N", "O", "P"): OS_KEYBOARD_INPUTS[inp] = []


InputType = Union[int, str]


@dataclass
class EventInfo:
    input: int
    event_value: float


class EventFuncWrapper:
    """
    keeping information about a function to be called upon a certain event, the function must take the event value as a parameter
    """
    def __init__(self, func: Callable = None, event_value_arg_name="event_value", **kwargs):
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


class MetaEventHandler(type):
    """
    Metaclass for EventHandlerConfig and all its child classes
    """
    def __init__(self, *args, **kwargs):
        self.NULL = -1
        if not hasattr(self, "INPUTS"):
            self.INPUTS = {}
        default_ids = {inp: -1 for inp in self.INPUTS}
        if not hasattr(self, "DEFAULT_IDS"):
            self.DEFAULT_IDS = default_ids
        else:
            self.DEFAULT_IDS = default_ids | self.DEFAULT_IDS


class EventHandlerConfig(metaclass=MetaEventHandler):
    """
    (abstract class)
    Handler for input events giving a continuous value (and also discrete values which are treated as continuous values)
    from a physical (or not) device, for example keyboard or gamepad.
    Comprises input ids, value offsets and amplitudes.
    Also allows to bind functions to be called upon the event of a certain input.
    """

    InputType = Union[int, str]
    
    def __init__(self, **kwargs):
        """
        :param kwargs: Must contain all the ids, offsets and amplitudes of the inputs that need to be set to a different than default value.
            All parameters need to be given as keyword arguments in the format [name of input]_id, [name of input]_offset and [name of input]_amplitude.
            Default ids are given by EventHandlerConfig.DEFAULT_IDS, default offsets are set to 0,  default amplitudes are set to 1.
        """
        self.input_ids : Dict[int, int] = {}
        self.input_offsets : Dict[int, float] = {}
        self.input_amplitudes : Dict[int, float] = {}
        for input, name in self.INPUTS.items():
            attr = f"{name}_id"
            self.input_ids[input] = kwargs.get(attr, self.DEFAULT_IDS[input])
            attr = f"{name}_offset"
            self.input_offsets[input] = kwargs.get(attr, 0.0)
            attr = f"{name}_amplitude"
            self.input_amplitudes[input] = kwargs.get(attr, 1.0)

        # dict of inputs correspondig to ids (if multiple inputs have the same id, for instance -1, the behaviour is undefined)
        self.reverse_id_dict : Dict[int, int] = {}
        for inp, id in self.input_ids.items():
            self.reverse_id_dict[id] = inp
        self.fast_id_finding = True

        # list of functions to be called on event for every input
        self.event_signals : Dict[int, List[Callable]] = {k: [] for k in self.INPUTS}

    def bind(self, input: EventHandlerConfig.InputType, func: Callable, event_value_arg_name: str, **kwargs) -> None:
        """
        Bind a function call to the trigger event of a certain input.
        :param input: an input type
        :param func: the function to bind (it needs to take the event value as a parameter)
        :param event_value_arg_name: the name of the function's parameter for the event value
        :param kwargs: other arguments of the function given as keyword arguments
        """
        if input in self.INPUTS:
            self.event_signals[input].append(EventFuncWrapper(func, event_value_arg_name, **kwargs))

    def find_input(self, id) -> int:
        """
        Determine the input that has the given id.
        :return: an input type
        """
        if self.fast_id_finding:
            return self.reverse_id_dict.get(id, self.NULL)
        for input, inp_id in self.input_ids.items():
            if inp_id == id:
                return input
        return self.NULL

    def valid_event(self, event) -> bool:
        """
        A check that the event is valid.
        Should be overriden to match the specific event type of the child class.
        """
        return True
    
    def get_event_id(self, event) -> int:
        """
        Extract a usable event id.
        Should be overriden to match the specific event type of the child class.
        """
        return self.NULL

    def get_event_raw_value(self, event) -> float:
        """
        Should be overriden
        """
        return 1.0
    
    def get_event_value(self, input: EventHandlerConfig.InputType, raw_value: float) -> float:
        offset = self.input_offsets[input]
        amplitude = self.input_amplitudes[input]
        event_value = (raw_value - offset) / amplitude
        return event_value
    
    def get_event_info(self, event) -> EventInfo:
        event_id = self.get_event_id(event)
        input = self.find_input(event_id)
        raw_value = self.get_event_raw_value(event)
        event_value = self.get_event_value(input, raw_value)
        return EventInfo(
            input=input,
            event_value=event_value
        )

    def handle_event(self, event) -> None:
        """
        Obtains the event info then calls then calls emit_signal for treatment of the event and calling of the binded functions
        """
        if not self.valid_event(event):
            return
        event_info = self.get_event_info(event)
        if event_info.input == self.NULL:
            return
        self.emit_signal(event_info)

    def emit_signal(self, event_info: EventInfo) -> None:
        """
        Call functions binded to input.
        :param input: an input type
        :param raw_value: raw value of the event, to be transformed to the desired range using the corresponding offset and amplitude of the config
        """
        for func in self.event_signals[event_info.input]:
            func(event_info.event_value)

    def read_inputs(self):
        """
        Override this method to read the inputs of the specific device
        """
        pass
    
    def loop(self):
        while True:
            self.read_inputs()

    def background_loop(self):
        thread = threading.Thread(target=self.loop, daemon=True)
        thread.start()
    
    def connect_events(src_handler: EventHandlerConfig, src_input: EventHandlerConfig.InputType, target_handler: EventHandlerConfig, target_input: EventHandlerConfig.InputType):
        """
        Connect an input from one handler to an input of another handler
        """
        if src_input not in src_handler.INPUTS or target_input not in target_handler.INPUTS:
            raise ValueError("input type not found in handler")
        
        def connection(value: float):
            event_info = EventInfo(
                input=target_input,
                event_value=value
            )
            target_handler.emit_signal(event_info)
        
        src_handler.bind(src_input, connection, "value")


class _KeyboardConfig(EventHandlerConfig):
    @classmethod
    def _set_inputs_as_class_attributes(cls):
        cls.INPUTS = {}
        for ind, key in enumerate(OS_KEYBOARD_INPUTS):
            if key.isalpha():
                name = key
            elif key.isnumeric():
                name = "_" + key
            else:
                continue
            setattr(cls, name, ind)
            cls.INPUTS[ind] = key


_KeyboardConfig._set_inputs_as_class_attributes()


class KeyboardConfig(_KeyboardConfig):
    """
    Handler for keyboard
    """
    
    DEFAULT_IDS = {inp: keyboard._canonical_names.normalize_name(key) for inp, key in _KeyboardConfig.INPUTS.items()}
    
    def __init__(self, **kwargs):
        keyboard.on_press(self.handle_event)
        super().__init__(**kwargs)
        
    def valid_event(self, event: keyboard._keyboard_event.KeyboardEvent) -> bool:
        """
        Overrides parent method
        """
        return True
    
    def get_event_id(self, event: keyboard._keyboard_event.KeyboardEvent) -> str:
        """
        Overrides parent method
        """
        return event.name


class GamePadConfig(EventHandlerConfig):
    """
    Handler for a gamepad using the evdev package
    """
    # TODO: make a virtual gamepad handler
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
    
    BUTTONS = {TRIANGLE: "triangle", SQUARE: "square", CIRCLE: "circle", CROSS: "cross", L1: "L1", R1: "R1", L3: "L3", R3: "R3", CREATE: "create", OPTIONS: "options", PS: "PS", TOUCHPAD: "touchpad"}
    ANALOG_TRIGGERS = {RH: "RH", RV: "RV", LH: "LH", LV: "LV", L2: "L2", R2: "R2", DIRH: "dirH", DIRV: "dirV"}
    INPUTS = BUTTONS | ANALOG_TRIGGERS
    DEFAULT_IDS = {L1: 310, R1: 311, L2: 2, R2: 5, PS: 316, LH: 0, LV: 1, RH: 3, RV: 4, DIRH: 16, DIRV: 17, TRIANGLE: 307, SQUARE: 308, CIRCLE: 305, CROSS: 304}

    def __init__(self, **kwargs):
        self.device = self.connect()
        kwargs |= self._get_additional_init_kwords()
        super().__init__(**kwargs)
        
    def valid_event(self, event: evdev.events.InputEvent) -> bool:
        """
        Overrides parent method
        """
        # ignore event type 0 (not sure what it corresponds to but results in weird behaviour)
        return event.type != 0
    
    def get_event_id(self, event: evdev.events.InputEvent) -> int:
        """
        Overrides parent method
        """
        return event.code
    
    def get_event_raw_value(self, event: evdev.events.InputEvent) -> float:
        """
        Overrides parent method
        """
        return event.value
    
    def connect(self) -> evdev.InputDevice:
        print("connecting to gamepad...")
        while True:
            for device in map(evdev.InputDevice, evdev.list_devices()):
                print(device)
                return device
            sleep(1)

    def read_inputs(self):
        """
        Overrides parent method
        """
        try:
            for event in self.device.read_loop():
                self.handle_event(event)

        except (TypeError, IOError) as e:
            raise
    
    def _get_additional_init_kwords(self):
        """
        Get custom parameters corresponding to the detected device (returns empty dict if the device name is not known)
        """
        if self.device.name == "Generic X-Box pad":
            return {
                "circle_id": 305,
                "triangle_id": 308,
                "square_id": 307,
                "cross_id": 304,
                "RH_amplitude": 2**15,
                "RV_amplitude": 2**15,
                "LH_amplitude": 2**15,
                "LV_amplitude": 2**15,
                "R2_amplitude": 2**8,
                "L2_amplitude": 2**8
            }
        elif self.device.name == "Wireless Controller":
            return {
                "circle_id": 305,
                "triangle_id": 307,
                "square_id": 308,
                "cross_id": 304,
                "RH_offset": 2**7,
                "RV_offset": 2**7,
                "LH_offset": 2**7,
                "LV_offset": 2**7,
                "RH_amplitude": 2**7,
                "RV_amplitude": 2**7,
                "LH_amplitude": 2**7,
                "LV_amplitude": 2**7,
                "R2_amplitude": 2**8,
                "L2_amplitude": 2**8
            }
        return {}


def test(val: float, label: str = "AAA"):
    print(label, val)


if __name__ == "__main__":
    controller = KeyboardConfig()
    controller.bind(KeyboardConfig._2, test, "val", label=2)
    controller.bind(KeyboardConfig.e, test, "val", label="e")
    controller.bind(KeyboardConfig.E, test, "val", label="E")
    while True:
        controller.read_inputs()
