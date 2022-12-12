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

        if(self.isBetween(closestPt)):
            return closestPt

        if(self.start.distTo(pt) < self.end.distTo(pt)):
            return self.start

        return self.end

    def intersects(self, line):
        x1 = self.start.x
        y1 = self.start.y
        x2 = self.end.x
        y2 = self.end.y
        x3 = line.start.x
        y3 = line.start.y
        x4 = line.end.x
        y4 = line.end.y

        denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
        if denom == 0: # parallel
            return False
        ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
        if ua < 0 or ua > 1: # out of range
            return False
        ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
        if ub < 0 or ub > 1: # out of range
            return False
        
        return True
