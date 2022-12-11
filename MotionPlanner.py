from abc import ABC, abstractmethod

class MotionPlanner(ABC):
    def __init__(self, obstacleMap):
        self.field = obstacleMap

    def generatePath(self, start, end, animate = True):
        pass

