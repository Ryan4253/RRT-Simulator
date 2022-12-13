from SearchSpace import SearchSpace
from Circle import Circle
from Point import Point
from RRTStar import RRTStar
from RRT import RRT
import matplotlib.pyplot as plt

a = SearchSpace(12, 12, [Circle(Point(6, 6), 3.9)], 0)
b = RRTStar(a, 0.1, 0.5, 2, 100, 2000)
c = RRT(a, 0.1, 0.5, 2000)
it, path, time = c.generatePath(Point(2, 6), Point(10, 6), True)
