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


@dataclass
class EventInfo:
    input: int
    event_value: float


class EventFuncWrapper:
    """
    keeping information about a function to be called upon a certain event, the function must take the event value as a parameter
    """
    def __init__(self, func: Callable = None, event_value_arg_name: str = "", **kwargs):
        """
        :param func: the function to call
        :param event_value_arg_name: the name of the function's parameter for the event value (if none is given, the event value won't be passed to the function)
        :param kwargs: additional arguments of the function
        """
        if func is None:
            func = lambda *args, **kwargs: None
        self.func = func
        self.event_value_arg_name = event_value_arg_name
        self.kwargs = kwargs
        if event_value_arg_name:
            self.kwargs[event_value_arg_name] = None
    
    def __call__(self, event_value) -> Any:
        if self.event_value_arg_name:
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
        self.input_offsets : Dict[int, float] = {self.NULL: 0.0}
        self.input_amplitudes : Dict[int, float] = {self.NULL: 1.0}
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

    def bind(self, input: EventHandlerConfig.InputType, func: Callable, event_value_arg_name: str = "", **kwargs) -> None:
        """
        Bind a function call to the trigger event of a certain input.
        :param input: an input type
        :param func: the function to bind (it needs to take the event value as a parameter)
        :param event_value_arg_name: the name of the function's parameter for the event value (if none is given, the event value won't be passed to the function)
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

    def background_loop(self, daemon: bool = True):
        thread = threading.Thread(target=self.loop, daemon=daemon)
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
