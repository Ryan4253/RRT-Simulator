from Shape import Shape
import matplotlib.pyplot as plt
from matplotlib import patches

class Circle(Shape):
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def lineIntersect(self, line, dist = 0):
        pt = line.closestPointTo(self.center)
        return self.center.distTo(pt) < (dist + self.radius)

    def pointIntersect(self, point, dist = 0):
        return point.distTo(self.center) < (dist + self.radius)

    def draw(self):
        plt.gca().add_patch(patches.Circle((self.center.x, self.center.y), self.radius, color='k'))
