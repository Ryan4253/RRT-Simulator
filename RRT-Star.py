from RRT import Node
from Point import Point
from Line import Line
from Rotation import Rotation
from random import random
from random import uniform
from time import sleep
from ObstacleMap import ObstacleMap
from Rectangle import  Rectangle
import math
from DiscretePath import DiscretePath
import matplotlib.pyplot as plt
from time import perf_counter
from Circle import Circle

class RRTStar:
    def __init__(self, searchSpace, stepSize, goalSampleRate, searchRadius, maxIteration):
        self.stepSize = stepSize
        self.goalSampleRate = goalSampleRate
        self.maxIteration = maxIteration
        self.searchSpace = searchSpace
        self.searchRadius = searchRadius
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
            
            neighbor = self.nearbyNodes(newNode.point)
            self.chooseParent(newNode, neighbor)
            self.rewire(newNode, neighbor)
            self.vertex.append(newNode)

            if (self.goal.parent is None and newNode.distTo(self.goal) <= self.stepSize and 
                not self.searchSpace.checkLineCollision(Line(newNode.point, self.goal.point))):
                self.goal.parent = newNode
                self.vertex.append(self.goal)

            if visualize:
                self.path = self.retrace()
                sleep(0.001)
                self.draw()

        self.path = self.retrace()
        return self.maxIteration, self.path, perf_counter() - tStart

    def getRandomPoint(self):
        if(random() < self.goalSampleRate):
            return self.goal.point
        
        return Point(uniform(0, self.searchSpace.x), uniform(0, self.searchSpace.y))

    def makeNewNode(self, parent, point):
        startPt = parent.point
        if(point == startPt):
            return None

        if(startPt.distTo(point) < self.stepSize):
            return Node(point, parent)
        
        return Node(startPt + (point-startPt).norm() * self.stepSize, parent)
    
    def nearestNode(self, point):
        return min(self.vertex, key = lambda node : node.point.distTo(point))

    def nearbyNodes(self, point):
        r = self.searchRadius
        return [node for node in self.vertex if node.distTo(point) <= r and not self.searchSpace.checkLineCollision(Line(point, node.point))]

    def chooseParent(self, newNode, neighbor):
        parent = newNode.parent
        for node in neighbor:
            if(newNode.distTo(parent) > newNode.distTo(node)):
                parent = node

        newNode.parent = parent

    def rewire(self, newNode, neighbor):
        for node in neighbor:
            if node.parent is not None and self.cost(node) > self.cost(newNode) + node.distTo(newNode):
               node.parent = newNode

    def retrace(self):
        if(self.goal.parent is None):
            return None

        path = [self.goal.point]
        parent = self.goal.parent
        while parent:
            path.insert(0, parent.point)
            parent = parent.parent
        
        return DiscretePath(path)

    def cost(self, node):
        if node.parent is None:
            return 0

        return node.distTo(node.parent) + self.cost(node.parent)

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


#a = ObstacleMap(12, 12, [Rectangle(Point(3, 10), Point(3.5, 0)), Rectangle(Point(8.5, 12), Point(9, 2))], 0)
a = ObstacleMap(12, 12, [Circle(Point(6, 6), 3.9)], 0)
b = RRTStar(a, 0.5, 0.1, 5, 1500)
it, path, time = b.generatePath(Point(2, 6), Point(10, 6), True)
print(path.getLength())
b.draw()
plt.show()