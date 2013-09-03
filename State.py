import random, math
from shapely.geometry import *

class State:
    # o: the initial asv position
    # booms: a set of boom with angle and length
    def __init__(self, o, booms=[]):
        self.polygon = None
        if type(o) == list: # to create initial and goal with a set of coords
            self.points = []
            for i, each in enumerate(o):
                if i == 0:
                    self.o = each
                self.points.append(Point(each[0],each[1]))
            self.booms = self._calAngle()
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

    def __str__(self):
        for i, each in enumerate(self.points):
            if i == 0:
                str = '%.5f %.5f' % (each.x,each.y)
                continue
            str += ' %.5f %.5f' % (each.x, each.y)
        return str

    def normaliseAngle(self, angle):
        while angle <= -math.pi:
            angle += 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def _calAngle(self):
        points = self.points
        booms = []
        angle = 0.0
        for i, each in enumerate(points):
            if i == len(points)-1: break;
            p0, p1 = points[i], points[i+1]
            length = p0.distance(p1)
            nextAngle = math.atan2(p1.y-p0.y, p1.x-p0.x)
            turning = self.normaliseAngle(nextAngle - angle)
            booms.append( (turning, length) )
            angle = nextAngle
        return booms

    def shape(self):
        if self.polygon: return self.polygon
        coords = []
        for each in self.points:
            coords.append( (each.x, each.y) )
        self.polygon = Polygon(coords)
        return self.polygon
        # return Polygon(coords)

    def isValid(self, minArea):
        coords = []
        for each in self.points:
            if each.x < 0 or each.x > 1 or each.y < 0 or each.y > 1:
                return False
            coords.append( (each.x, each.y) )
        self.polygon = polygon = Polygon(coords)
        if not polygon.is_valid: return False
        if len(self.points) != 3:
            if not self.checkConvex(polygon): return False

        return polygon.area >= minArea

    def checkConvex(self, polygon):
        n = len(self.points)
        p0, p1, p2 = self.points[0:3]
        pn_1, pn = self.points[n-2:]
        zcross1 = (pn.x-pn_1.x) * (p0.y-pn.y) - (p0.x-pn.x) * (pn.y-pn_1.y)
        zcross2 = (p1.x-p0.x) * (p2.y-p1.y) - (p2.x-p1.x) * (p1.y-p0.y)
        zcross3 = (p0.x-pn.x) * (p1.y-p0.y) - (p1.x-p0.x) * (p0.y-pn.y)
        return (zcross1<0 and zcross2<0 and zcross3<0) or (zcross1>0 and zcross2>0 and zcross3>0)

    def checkBoundary(self):
        for each in self.points:
            if each.x < 0 or each.x > 1 or each.y < 0 or each.y > 1:
                return False
        return True

    def distance(self, other):
        return self.shape().centroid.distance(other.shape().centroid)
        # return self.shape().distance(other.shape())

    def totalDistance(self, other):
        totalDist = 0.0
        for i, each in enumerate(self.points):
            si = self.points[i]
            di = other.points[i]
            dist = si.distance(di)
            totalDist += dist
        return totalDist 
    def maxDistance(self, other):
        maxdist = 0.0
        for i, each in enumerate(self.points):
            si = self.points[i]
            di = other.points[i]
            dist = si.distance(di)
            if dist > maxdist:
                maxdist = dist
        return maxdist
    def minDistance(self, other):
        for i, each in enumerate(self.points):
            si = self.points[i]
            di = other.points[i]
            dist = si.distance(di)
            if i == 0:
                mindist = dist
                continue
            if dist < mindist:
                mindist  = dist
        return mindist    

    def initDistance(self, other):
        return self.points[0].distance(other.points[0])

    def within(self, other):
        return self.shape().centroid.within(other.shape())

    def printBooms(self):
        s = []
        for each in self.booms:
            s.append( (math.degrees(each[0]), each[1]) )
        # print s
        return s

if __name__ == '__main__':
    init = State([(0.185,0.240), (0.150,0.180), (0.220,0.180)])
    s0 = State( [(0.18500, 0.28100), (0.12200, 0.25000), (0.10600, 0.18200), (0.15000, 0.12700), (0.22000, 0.12700), (0.26400, 0.18200), (0.24800, 0.25000)] )
    s1 = State( [(0.23674, 0.18377), (0.30252, 0.21163), (0.31631, 0.26301), (0.32226, 0.32635), (0.32822, 0.39365), (0.27717, 0.36204), (0.25013, 0.30399)] )
    s2 = State( [(0.33899, 0.33085), (0.33474, 0.38728), (0.29954, 0.43654), (0.24330, 0.45355), (0.19400, 0.42616), (0.15455, 0.37255), (0.20219, 0.31921)] )
    s5 = State( [(0.83240, 0.16494), (0.86844, 0.23017), (0.90366, 0.29574), (0.83380, 0.30965), (0.77209, 0.30027), (0.77994, 0.25083), (0.80699, 0.18717)] )
    s6 = State( [(0.88700, 0.07100), (0.88000, 0.12200), (0.85700, 0.16900), (0.81500, 0.20000), (0.77300, 0.16900), (0.75000, 0.12200), (0.74300, 0.07100)] )
    s7 = State( [(0.32184, 0.55258), (0.26281, 0.53311), (0.23430, 0.48809), (0.22138, 0.43325), (0.22077, 0.37438), (0.26811, 0.33909), (0.32072, 0.31852)] )
    s8 = State( [(0.49590, 0.58733), (0.49010, 0.64080), (0.45582, 0.67737), (0.40858, 0.64187), (0.36358, 0.59653), (0.31943, 0.53842), (0.38098, 0.54232)] )

    s9 = State([(0.65929, 0.36677), (0.69401, 0.30389), (0.75153, 0.27526), (0.81408, 0.31322), (0.76386, 0.31092), (0.73403, 0.24603), (0.77186, 0.19973)] )
    s10 = State([(0.88164, 0.36144), (0.84519, 0.30423), (0.84918, 0.25148), (0.90156, 0.20323), (0.91232, 0.27312), (0.92215, 0.33772), (0.91678, 0.40629)] )
    print s9.distance(s10)
    print s9.shape().is_valid
    print s10.shape().is_valid
    # print s0.distance(s1)
    # print s1.distance(s2)
# 
    # print s5.distance(s6)
    # print s7.distance(s8)
