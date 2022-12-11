from abc import ABC, abstractmethod

class Shape(ABC):
    @abstractmethod
    def intersects(self, line, dist=0):
        pass

    def draw(self):
        pass