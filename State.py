import random, math
from shapely.geometry import *

class State:
    # o: the initial asv position
    # booms: a set of boom with angle and length
    def __init__(self, o, booms=[]):
        if type(o) == list: # to create initial and goal with a set of coords
            self.points = []
            self.booms = booms
            for i, each in enumerate(o):
                if i == 0:
                    self.o = each
                self.points.append(Point(each[0],each[1]))
        else:
            x, y = self.o = o
            self.booms = booms
            self.points = [ Point(x, y) ] # the initial asv
            angle = 0.0
            for boom in booms:
                # angle offset between the current boom and its previous one
                angleDelta = boom[0] 
                length = boom[1]
                angle += angleDelta 
                x += length*math.cos(angle)
                y += length*math.sin(angle)
                self.points.append( Point(x, y) )

        # n = len(self.points) - 1
        # self.MIN_AREA = ( (0.007 * n) ** 2 ) * math.pi

    def __str__(self):
        for i, each in enumerate(self.points):
            if i == 0:
                str = '%.3f %.3f' % (each.x,each.y)
                continue
            str += ' %.3f %.3f' % (each.x, each.y)
        return str

    def shape(self):
        coords = []
        for each in self.points:
            coords.append( (each.x, each.y) )
        return Polygon(coords)

    def isValid(self, minArea):
        coords = []
        for each in self.points:
            if each.x < 0 or each.x > 1 or each.y < 0 or each.y > 1:
                return False
            coords.append( (each.x, each.y) )
        polygon = Polygon(coords)
        if not polygon.is_valid: return False
        if len(self.points) != 3:
            if not self.checkConvex(polygon): return False

        return polygon.area >= minArea

    def checkConvex(self, polygon):
        n = len(self.points)
        # for i in range(n):
        #     if i == n-2:
        #         p0, p1, p2 = self.points[i], self.points[i+1], self.points[0]
        #     elif i == n-1:
        #         p0, p1, p2 = self.points[i], self.points[0], self.points[1]
        #     else:
        #         p0, p1, p2 = self.points[i], self.points[i+1], self.points[i+2]
        #     zcross1 = (p1.x-p0.x) * (p2.y-p1.y) - (p2.x-p1.x) * (p1.y-p0.y)
            # if i == 0: continue
            # if (zcross1<0 and zcross2<0) or (zcross1>0 and zcross2>0)

        p0, p1, p2 = self.points[0:3]
        pn_1, pn = self.points[n-2:]
        zcross1 = (pn.x-pn_1.x) * (p0.y-pn.y) - (p0.x-pn.x) * (pn.y-pn_1.y)
        zcross2 = (p1.x-p0.x) * (p2.y-p1.y) - (p2.x-p1.x) * (p1.y-p0.y)
        zcross3 = (p0.x-pn.x) * (p1.y-p0.y) - (p1.x-p0.x) * (p0.y-pn.y)
        return (zcross1<0 and zcross2<0 and zcross3<0) or (zcross1>0 and zcross2>0 and zcross3>0)

    def distance(self, other):
        sourceO = self.points[0]
        destO = other.points[0]
        return abs(sourceO.x - destO.x) + abs(sourceO.y - destO.y)
        # return abs(self.o[0] - other.o[0]) + abs(self.o[1] - other.o[1])
        # return self.shape().distance(other.shape())

