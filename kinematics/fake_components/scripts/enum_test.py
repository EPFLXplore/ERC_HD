from __future__ import annotations
from typing import Any, TypeVar, Generic

T = TypeVar('T')


class EnumOld:
    # kinda overkill but for fun to mimic a C-style enum + some extra functionalities

    def __init__(self, **kwargs):
        self.items = kwargs
        self.slots = list(kwargs)
        self.slot_values = [kwargs[slot] for slot in self.slots]
        #print(self.slots)
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
        i = (self.slot_values.index(val)+1) % len(self.slots)
        return self.slot_values[i]


def Enum(**kwargs: T) -> type:
    """
    *** very overkill and very useless but I was bored ***
    Tries to mimic a C-style enum with some additional useful properties.
    :param kwargs: the members of the enum
    :return: A class type having as class attributes instances of that class corresponding to the members of the enum.
        Thanks to the class being constructed on a custom metaclass, iteration, len computation and item query can be performed directly on the enum class object.
    """
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
        
        def _next(self, obj: EnumClass):
            for i, slot in enumerate(self.__SLOTS):
                if slot == obj:
                    break
            return self.__SLOTS[(i+1)%len(self.__SLOTS)]
        
        def register_instance(self, name: str, value: T):
            setattr(self, name, value)
            self.__SLOTS.append(getattr(self, name))

    class EnumClass(metaclass=EnumMetaClass):
        def __init__(self, name: str, value: T):
            self.name = name
            self.value = value
            EnumClass.register_instance(name, self)

        def __eq__(self, other: Any):
            if isinstance(other, EnumClass):
                return other.value == self.value
            elif isinstance(other, type(self.value)):
                return other == self.value
            return False

        def next(self):
            return EnumClass._next(self)

        def __repr__(self):
            return f"<name: {self.name}, value: {self.value}>"

    for name, value in kwargs.items():
        EnumClass(name, value)
    EnumClass.__new__ = lambda *args, **kwargs: None    # class won't be instantiable anymore
    return EnumClass



if __name__ == "__main__":
    Fruits = Enum(
        apple = 0,
        orange = 1,
        banana = 2,
    )
