from Point import Point
from Line import Line
from Shape import Shape
import matplotlib.pyplot as plt
from matplotlib import patches

class Rectangle(Shape):
    def __init__(self, topLeft, bottomRight):
        self.topLeft = topLeft
        self.topRight = Point(bottomRight.x, topLeft.y)
        self.bottomLeft = Point(topLeft.x, bottomRight.y)
        self.bottomRight = bottomRight

    def intersects(self, line, dist = 0):
        p1 = Point(self.topLeft.x-dist, self.topLeft.y+dist)
        p2 = Point(self.topRight.x+dist, self.topRight.y+dist)
        p3 = Point(self.bottomRight.x+dist, self.bottomRight-dist)
        p4 = Point(self.bottomLeft.x-dist, self.bottomLeft.y-dist)
        

        l1 = Line(p1, p2)
        l2 = Line(p2, p3)
        l3 = Line(p3, p4)
        l4 = Line(p4, p1)

        return not (line.intersects(l1) or line.intersects(l2) or line.intersects(l3) or line.intersects(l4))

    def draw(self):
        x = self.topRight.x - self.bottomLeft.x
        y = self.topRight.y - self.bottomLeft.y
        plt.gca().add_patch(patches.Rectangle((self.bottomLeft.x, self.bottomLeft.y), x, y, color='k'))