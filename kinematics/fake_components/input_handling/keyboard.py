from .handler import *
import keyboard
import keyboard._keyboard_event
keyboard._os_keyboard.init()

OS_KEYBOARD_INPUTS = copy.deepcopy(keyboard._os_keyboard.from_name)
OS_KEYBOARD_INPUTS: Dict[str, List[Tuple[int, Tuple[str, ...]]]]
for inp in ("A", "D", "M", "N", "O", "P"): OS_KEYBOARD_INPUTS[inp] = []


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
        keyboard.hook(self.handle_event)
        super().__init__(**kwargs)
    
    def get_event_id(self, event: keyboard._keyboard_event.KeyboardEvent) -> str:
        """
        Overrides parent method
        """
        return event.name
    
    def get_event_raw_value(self, event: keyboard._keyboard_event.KeyboardEvent) -> float:
        """
        Overrides parent method
        """
        return 1.0 if event.event_type == keyboard.KEY_DOWN else 0.0


if __name__ == "__main__":
    config = KeyboardConfig()
    config.background_loop()
