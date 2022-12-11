from abc import ABC, abstractmethod

class MotionPlanner(ABC):
    def __init__(self, obstacleMap):
        self.field = obstacleMap

    staticmethod
    def samplePoint(self):
        

    def generatePath(self, pStart, pEnd, animation = True):
        pass

