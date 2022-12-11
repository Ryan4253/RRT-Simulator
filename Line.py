from Point import Point

class Line:
    def __init__(self, p1, p2):
        self.start = p1
        self.end = p2

    def interpolate(self, t):
        if(t <= 0):
            return self.start

        if(t >= 1):
            return self.end
        
        return self.start + (self.end - self.start)*t

    def length(self):
        return self.start.distTo(self.end)

    def proj(self, pt):
        v1 = pt - self.start
        v2 = self.end - self.start
        pt = v1.proj(v2)

        return self.start + pt
    
    def isBetween(self, pt):
        v = self.start - pt
        w = self.end - pt

        return abs(v.cross(w)) < 0.0001 and v.dot(w) < 0

    def closestPointTo(self, pt):
        closestPt = self.proj(pt)
        bl = self.isBetween(closestPt)
        print(bl)
        if(self.isBetween(closestPt)):
            return closestPt

        if(self.start.distTo(pt) < self.end.distTo(pt)):
            return self.start

        return self.end

    def intersects(self, line):
        return True