from SearchSpace import SearchSpace
from Circle import Circle
from Point import Point
from RRTStar import RRTStar
import matplotlib.pyplot as plt

a = SearchSpace(12, 12, [Circle(Point(6, 6), 3.9)], 0)
b = RRTStar(a, 0.1, 0.5, 2, 500, 2000)
it, path, time = b.generatePath(Point(2, 6), Point(10, 6), False)
print(path.getLength())
b.draw()
plt.show()