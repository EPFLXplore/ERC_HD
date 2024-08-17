from abc import ABC, abstractmethod


class MonocularCameraInterface(ABC):
    @abstractmethod
    def get_image(self):
        pass
