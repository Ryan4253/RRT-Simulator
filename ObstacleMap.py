from Point import Point
from Line import Line
from Rectangle import Rectangle
from Circle import Circle
import matplotlib.pyplot as plt

class ObstacleMap:
    def __init__(self, x, y, obstacleList, radius=0):
        self.x = x
        self.y = y
        self.obstacleList = obstacleList
        self.radius = radius
        p1 = Point(radius, radius)
        p2 = Point(radius, y-radius)
        p3 = Point(x-radius, y-radius)
        p4 = Point(x-radius, radius)
        self.edgeList = [Line(p1, p2), Line(p2, p3), Line(p3, p4), Line(p4, p1)]
    
    def isInBounds(self, pt):
        return (0 <= pt.x <= self.x) and (0 <= pt.y <= self.y)

    def checkLineCollision(self, line):
        for obstacle in self.obstacleList:
            if(obstacle.lineIntersect(line, self.radius)):
                return True

        for edge in self.edgeList:
            if(edge.intersects(line)):
                return True
            
        return False

    def checkPointCollision(self, point):
        for obstacle in self.obstacleList:
            if(obstacle.pointIntersect(point, self.radius)):
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

