from Point import Point

class Node:
    def __init__(self, pt, par = None):
        self.point = pt
        self.parent = par