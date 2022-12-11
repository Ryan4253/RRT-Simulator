from abc import ABC, abstractmethod

class Shape(ABC):
    @abstractmethod
    def lineIntersect(self, line, dist = 0):
        pass

    def pointIntersect(self, point, dist = 0):
        pass

    def draw(self):
        pass