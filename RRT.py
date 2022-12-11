from Point import Point
from Node import Node
from Line import Line
from DiscretePath import DiscretePath
from MotionPlanner import MotionPlanner
from ObstacleMap import ObstacleMap
from Circle import Circle
from random import random
from random import uniform
from time import sleep
from time import perf_counter
import matplotlib.pyplot as plt

class RRT(MotionPlanner):
    def __init__(self, obstacleMap, stepSize, goalConvergenceRate, maxIteration):
        self.stepSize = stepSize
        self.goalConvergenceRate = goalConvergenceRate
        self.maxIteration = maxIteration
        self.obstacleMap = obstacleMap
        self.start = None
        self.end = None
        self.tree = None

    def generatePath(self, start, target, visualize = True):
        self.start = Node(start)
        self.end = Node(target)
        self.tree = [self.start]
        self.path = None
        tStart = perf_counter()

        if(self.obstacleMap.checkPointCollision(start) or self.obstacleMap.checkPointCollision(target)):
            return -1, None, -1

        for i in range(self.maxIteration):
            randPt = self.sample()
            closestNode, closestNodeId = self.findClosestNode(randPt)
            newNode = closestNode.point + (randPt-closestNode.point).norm() * self.stepSize 

            if(self.obstacleMap.checkLineCollision(Line(closestNode.point, newNode))):
                continue

            self.tree.append(Node(newNode, closestNodeId))

            if visualize:
                sleep(0.01)
                self.draw()

            if(newNode.distTo(self.end.point) < self.stepSize):
                if(not self.obstacleMap.checkLineCollision(Line(newNode, self.end.point))):
                    self.tree.append(Node(self.end.point, len(self.tree)-1))
                    self.path = self.retrace()
                    self.draw()
                    return i+1, self.path, perf_counter() - tStart

        return self.maxIteration, None, perf_counter() - tStart

    def sample(self):
        if(random() < self.goalConvergenceRate):
            return self.end.point
        
        return Point(uniform(0, self.obstacleMap.x), uniform(0, self.obstacleMap.y))

    def findClosestNode(self, point):
        idx = min(range(len(self.tree)), key = lambda i : self.tree[i].point.distTo(point))
        return self.tree[idx], idx

    def retrace(self):
        ret = []
        index = len(self.tree)-1
        while True:
            ret.insert(0, self.tree[index].point)
            if(self.tree[index].parent is None):
                return DiscretePath(ret)
            index = self.tree[index].parent           

    def draw(self):
        # Initialize
        plt.clf()

        # Draw Obstacles
        self.obstacleMap.draw()

        # Draw Start & End Point
        plt.plot(self.start.point.x, self.start.point.y, "or", markersize = 15)
        plt.plot(self.end.point.x, self.end.point.y, "og", markersize = 15)

        # Draw Tree
        for node in self.tree:
            if(node.parent is not None):
                plt.plot([node.point.x, self.tree[node.parent].point.x], [node.point.y, self.tree[node.parent].point.y], '-b')

        # Draw Path
        if(self.path is not None):
            for i in range(self.path.size()-1):
                plt.plot([self.path[i].x, self.path[i+1].x], [self.path[i].y, self.path[i+1].y], '-g')
            plt.pause(0.01)
            plt.show()

        plt.pause(0.01)


a = ObstacleMap(12, 12, [Circle(Point(3, 3), 1), Circle(Point(12, 12), 2)], 0.5)
b = RRT(a, 0.5, 0.1, 3000)
it, path, t = b.generatePath(Point(6, 6), Point(1, 1))

print("Iteration Count: ", it)
print("Path Length: ", path.getLength())
print("Time Elapsed: ", t)

    
