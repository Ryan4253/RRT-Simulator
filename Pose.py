from Rotation import Rotation
from Point import Point
import math
import copy

class Pose: 

    def __init__(self, a, b = None, c = None):
        if(isinstance(a, Pose)):
            self.point = copy.deepcopy(a.point)
            self.rotation = copy.deepcopy(a.rotation)

        elif(isinstance(b, Rotation)):
            self.point = copy.deepcopy(a)
            self.rotation = copy.deepcopy(b)
            
        else:
            self.point = Point(a, b)
            self.rotation = copy.deepcopy(c)

    def X(self):
        return self.point.x

    def Y(self):
        return self.point.y

    def Theta(self):
        return self.rotation.theta

    def __eq__(self, rhs):
        return self.point == rhs.point and self.rotation == rhs.rotation

    def __ne__(self, rhs):
        return not(self == rhs)