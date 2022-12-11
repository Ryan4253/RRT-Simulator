from Point import Point
from Line import Line
from Rectangle import Rectangle
from Circle import Circle
import matplotlib.pyplot as plt

class ObstacleMap:
    def __init__(self, x, y, obstacleList, radius = 0):
        self.x = x
        self.y = y
        self.obstacleList = obstacleList
        self.radius = radius
    
    def isInBounds(self, pt):
        return (pt.x >= 0 and pt.x <= self.x) and (pt.y >= 0 and pt.y <= self.y)

    def checkCollision(self, line):
        for obstacle in self.obstacleList:
            if(obstacle.intersects(line, self.radius)):
                return True

        return False

    def draw(self):
        plt.title('Field')
        plt.axis('square')
        plt.xlabel('X (Feet)')
        plt.ylabel('Y (Feet)')
        plt.gca().set_xlim((0, self.x))
        plt.gca().set_ylim((0, self.y))

        for obstacle in self.obstacleList:
            obstacle.draw()

a = ObstacleMap(12, 12, [Circle(Point(3, 3), 1), Circle(Point(12, 12), 2), Rectangle(Point(2, 12), Point(2.5, 8))])
a.draw()
plt.show()