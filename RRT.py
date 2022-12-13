from Node import Node
from Line import Line
from DiscretePath import DiscretePath
from MotionPlanner import MotionPlanner
from random import random
from time import sleep
from time import perf_counter
import matplotlib.pyplot as plt

class RRT(MotionPlanner):
    def __init__(self, searchSpace, goalSampleRate = 0.1, stepSize = 1, maxIter = 10000):
        self.stepSize = stepSize
        self.goalSampleRate = goalSampleRate
        self.maxIteration = maxIter
        self.searchSpace = searchSpace
        self.start = None
        self.goal = None
        self.vertex = None
        self.path = None
    
    def generatePath(self, start, goal, visualize = True):
        self.start = Node(start)
        self.goal = Node(goal)
        self.vertex = [self.start]
        tStart = perf_counter()

        if(self.searchSpace.checkPointCollision(start) or self.searchSpace.checkPointCollision(goal)):
            return -1, None, -1

        for i in range(self.maxIteration):
            sample = self.getRandomPoint()
            nearestNode = self.nearestNode(sample)
            newNode = self.makeNewNode(nearestNode, sample)

            if(newNode is None or self.searchSpace.checkLineCollision(Line(nearestNode.point, newNode.point))):
                continue

            self.vertex.append(newNode)

            if (self.goal.parent is None and newNode.distTo(self.goal) <= self.stepSize and 
                not self.searchSpace.checkLineCollision(Line(newNode.point, self.goal.point))):
                self.goal.parent = newNode
                self.path = self.retrace()
                if visualize:
                    self.draw()
                    plt.show()
                return i + 1, self.path, perf_counter() - tStart

            if visualize:
                sleep(0.001)
                self.draw()

        self.path = self.retrace()
        return self.maxIteration, self.path, perf_counter() - tStart

    def getRandomPoint(self):
        if(random() < self.goalSampleRate):
            return self.goal.point
        
        return self.searchSpace.randomPoint()

    def makeNewNode(self, parent, point):
        if(point == parent.point):
            return None

        if(parent.distTo(point) < self.stepSize):
            return Node(point, parent)
        
        return Node(parent.point + (point-parent.point).norm() * self.stepSize, parent)
    
    def nearestNode(self, point):
        return min(self.vertex, key = lambda node : node.distTo(point))

    def retrace(self):
        if(self.goal.parent is None):
            return None

        path = [self.goal.point]
        parent = self.goal.parent
        while parent:
            path.insert(0, parent.point)
            parent = parent.parent
        
        return DiscretePath(path)

    def draw(self):
        # Initialize
        plt.clf()

        # Draw Obstacles
        self.searchSpace.draw()

        # Draw Start & End Point
        plt.plot(self.start.point.x, self.start.point.y, "ob", markersize = 10)
        plt.plot(self.goal.point.x, self.goal.point.y, marker = "*", color = 'y', markersize = 15)

        # Draw Tree
        for node in self.vertex:
            if(node.parent is not None):
                plt.plot([node.point.x, node.parent.point.x], [node.point.y, node.parent.point.y], '-r')

        # Draw Path
        if(self.path is not None):
            for i in range(self.path.size()-1):
                plt.plot([self.path[i].x, self.path[i+1].x], [self.path[i].y, self.path[i+1].y], '-g', linewidth = 3)

        plt.pause(0.001)