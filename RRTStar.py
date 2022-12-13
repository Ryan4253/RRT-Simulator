from Node import Node
from Line import Line
from DiscretePath import DiscretePath
from MotionPlanner import MotionPlanner
from random import random
from time import sleep
from time import perf_counter
import matplotlib.pyplot as plt

class RRTStar(MotionPlanner):
    def __init__(self, searchSpace, goalSampleRate = 0.1, stepSize = 1, searchRadius = 1, optimizationIter = 500, maxIter = 10000):
        self.stepSize = stepSize
        self.goalSampleRate = goalSampleRate
        self.maxIteration = maxIter
        self.searchSpace = searchSpace
        self.searchRadius = searchRadius
        self.optimizationIteration = optimizationIter
        self.start = None
        self.goal = None
        self.vertex = None
        self.path = None
    
    def generatePath(self, start, goal, visualize = True):
        self.start = Node(start)
        self.goal = Node(goal)
        self.vertex = [self.start]
        tStart = perf_counter()
        optiIter = 0

        if(self.searchSpace.checkPointCollision(start) or self.searchSpace.checkPointCollision(goal)):
            return -1, None, -1

        for i in range(self.maxIteration):
            sample = self.getRandomPoint()
            nearestNode = self.nearestNode(sample)
            newNode = self.makeNewNode(nearestNode, sample)

            if(newNode is None or self.searchSpace.checkLineCollision(Line(nearestNode.point, newNode.point))):
                continue
            
            neighbor = self.nearbyNodes(newNode)
            self.chooseParent(newNode, neighbor)
            self.rewire(newNode, neighbor)
            self.vertex.append(newNode)

            if (self.goal.parent is None and newNode.distTo(self.goal) <= self.stepSize and 
                not self.searchSpace.checkLineCollision(Line(newNode.point, self.goal.point))):
                self.goal.parent = newNode
                self.vertex.append(self.goal)

            if self.goal.parent is not None:
                optiIter += 1

            if optiIter >= self.optimizationIteration:
                if visualize:
                    self.draw()
                    plt.show()
                return i+1, self.path, perf_counter() - tStart

            if visualize:
                self.path = self.retrace()
                sleep(0.001)
                self.draw()

        self.path = self.retrace()
        return self.maxIteration, self.path, perf_counter() - tStart

    def getRandomPoint(self):
        if(random() < self.goalSampleRate):
            return self.goal.point
        
        return self.searchSpace.randomPoint()

    def cost(self, node):
        if node.parent is None:
            return 0

        return node.distTo(node.parent) + self.cost(node.parent)

    def makeNewNode(self, parent, point):
        if(point == parent.point):
            return None

        if(parent.distTo(point) < self.stepSize):
            return Node(point, parent)
        
        return Node(parent.point + (point-parent.point).norm() * self.stepSize, parent)
    
    def nearestNode(self, point):
        return min(self.vertex, key = lambda node : node.distTo(point))

    def nearbyNodes(self, node):
        r = self.searchRadius
        return [nd for nd in self.vertex if nd.distTo(node) <= r and not self.searchSpace.checkLineCollision(Line(node.point, nd.point))]

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
