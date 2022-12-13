from Point import Point

class Node:
    def __init__(self, pt, par = None, cost = 0):
        self.point = pt
        self.parent = par

    def distTo(self, node):
        return node.distTo(self.point)
