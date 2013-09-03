
import math
from motionplanning.State import State
from motionplanning.Sampler import Sampler
from motionplanning.Obstacle import Obstacle
from motionplanning.Timer import Timer
# from graph.prioritydict import priority_dict

class Vertex:
    def __init__(self, state):
        self.index = -1
        self.state = state

        self.predecessor = None
        self.dtime = -1
        self.ftime = -1
        self.cost = float('inf')

    def getIndex(self):
        return self.index
    def setIndex(self, index):
        self.index = index

    def getPredecessor(self):
        return self.predecessor
    def setPredecessor(self, v):
        self.predecessor = v

    def estimateCost(self, goal):
        # return self.state.distance(goal.state)
        return self.state.maxDistance(goal.state)

    def isAncestor(self, s):
        p = self.getPredecessor()
        if p is None: return False
        elif p == s: return True 
        return p.isAncestor(s)
     
    def __lt__(self, other):
        return self.cost < other.cost
    def __hash__(self):
        return hash(self.index)
    def __eq__(self, other):
        return self.index == other.index

    def __repr__(self):
        return self.label
    def __str__(self):
        return str(self.state)

class Edge:
    def __init__(self, source, destination, weight=1.0):
        self.source = source
        self.destination = destination
        self.weight = weight
    
    def getSource(self):
        return self.source

    def getDestination(self):
        return self.destination

    def getWeight(self):
        return self.weight
    def setWeight(self, w):
        self.weight = w

    def reverse(self):
        return Edge(self.destination, self.source, self.weight)

    def __str__(self):
        return '%s->%s [%d]' % (str(self.source), str(self.destination), self.weight)

class RoadMap:
    class VEntry:
        def __init__(self, v):
            self.vert = v
            self.edges = []

    class EdgeIter:
        def __init__(self, v, graph):
            self.graph = graph
            self.edgesTo = graph.getVEntry(v).edges

        def __iter__(self):
            return iter(self.edgesTo)

    def __init__(self, asvNum, obstacles, N=50, radius=0.1):
        self.vertices = []
        self.curIndex = -1
        self.asvNum = asvNum
        self.obstacles = obstacles
        self.N = N
        self.radius = radius

        self.tempstateNum = 0
        self.tempstateTime = 0.0
        self.fhandler = open('profile.log', 'a')

    def build(self):
        N = self.N
        sampler = self.sampler = Sampler(self)
        print '\nbuilding road map with radius %.3f...' % self.radius
        with Timer() as t:
            for i in range(N):
                # generate a valid sample state
                state = sampler.sampling() 
                self.addVertex( Vertex(state) )
        print 'total samples: %d, time cost: %.2f s' % (self.size(), t.secs)
        self.connect()
    
    def connect(self):
        size = self.size()
        radius = self.radius
        edges = 0
        conn = 0
        with Timer() as t:
            for i, u in enumerate(self):
                for j in xrange(i+1, size):
                    conn += 1
                    dest = self.getVertex(j)
                    dist = u.state.distance(dest.state) 
                    # dist = u.state.maxDistance(dest.state) 
                    if dist > radius:
                        continue
                    if self.binary_reachable(u.state, dest.state):
                        edges += 1
                        weight = u.state.totalDistance(dest.state) 
                        self.addEdge(u, dest, weight)
        print 'total edges: %d' % edges
        print 'connect trys: %d, total connecting time: %.2f s' % (conn, t.secs)
        print 'generated intermediate states: %d' % (self.tempstateNum)

    def incremental_reachable(self, source, dest, dist):
        # reachable = True
        with Timer() as t:
            stepNum = int(math.ceil(dist / 0.001) )
            for i in range(stepNum):
                tempState = self.sampler.interpolate(source.state, dest.state, i+1)
                self.tempstateNum += 1
                if self.isCollison(tempState):
                    reachable = False
                    break
        self.tempstateTime += t.secs
        return reachable

    def binary_reachable(self, source, dest):
        Q = [(source, dest)]
        while Q:
            sn, dn = Q.pop(0)
            dist = sn.distance(dn)
            if dist < 0.006:
                continue
            stepNum = int( math.ceil(dist / 0.001) )
            t = stepNum / 2 
            tempState = self.sampler.interpolate_adv(sn, dn, t, stepNum)
            self.tempstateNum += 1
            if self.isCollison(tempState) or not tempState.checkBoundary():
                return False
            Q.append( (sn, tempState) )
            Q.append( (tempState, dn) )
        return True

    def recursive_reachable(self, source, dest):
        try:
            dist = source.distance(dest)
            stepNum = int(math.ceil(dist / 0.001) )
            if stepNum < 2 or dist < 0.001:
                return True
            t = stepNum / 2 
            # middle state in between
            tempState = self.sampler.interpolate_adv(source, dest, t, stepNum)
            self.tempstateNum += 1
            if self.isCollison(tempState):
                return False

            l = self.reachable(source, tempState)
            if l == False: 
                return False
            else:
                return self.reachable(tempState, dest)
        except:
            pass
            # print source
            # print tempState
            # print dest

    def isCollison(self, state):
        for obs in self.obstacles:
            if obs.collide(state):
                return True
        return False

    def size(self):
        return len(self.vertices)

    def getVertex(self, index):
        try:
            result = self.vertices[index].vert
        except IndexError:
            result = None
        return result

    def getVEntry(self, v):
        return self.vertices[v.getIndex()]

    def degree(self, v):
        return len(self.getVEntry(v).edges)

    def addVertex(self, v):
        v.setIndex(self.size())
        self.vertices.append(RoadMap.VEntry(v))

    def addEdge(self, source, dest, weight):
        newedge = Edge(source, dest, weight)
        self.getVEntry(source).edges.append(newedge)
        self.getVEntry(dest).edges.append( newedge.reverse() )
        return newedge

    def adjcent(self, u):
        return self.EdgeIter(u, self)

    def next(self):
        self.curIndex += 1
        try:
            result = self.vertices[self.curIndex].vert
        except IndexError:
            self.curIndex = -1
            raise StopIteration
        return result

    def __iter__(self):
        return self
        # return iter(self.vertices)

if __name__ == '__main__':
    coords = [ [(0.375,0.000), (0.625,0.000), (0.625,0.480), (0.375,0.480)], \
             [(0.375, 0.520), (0.625, 0.520), (0.625, 1.000), (0.375, 1.000)] ]
    obs1 = Obstacle(coords[0])
    obs2 = Obstacle(coords[1])
    init = State([(0.185,0.240), (0.150,0.180), (0.220,0.180)])
    G = RoadMap(asvNum=3, obstacles=[obs1, obs2], N=200, radius=0.15)
    G.addVertex( Vertex(init) )
    sam = Sampler(G)
    # s1 = sam.sampling()
    # s2 = sam.sampling()
    # s3 = sam.sampling()
    # dist = s1.distance(s2)
    # maxdist = s1.maxDistance(s2)
    # steps = int(math.ceil(maxdist / 0.001) )
    # i = steps / 2
    # print 'distance: %.3f' % dist
    # print 'max distance: %.3f, %d steps' % (maxdist, steps)
    # print s1.printBooms()
    # print '\n'

    # temp = sam.interpolate_adv(s1,s2,i,steps)
    # print temp.printBooms()
    # temp = sam.interpolate_adv(s1,s2,i/2,steps)
    # print temp.printBooms()
    # for i in range(15):
    #     temp = sam.interpolate_adv(s1,s2,i+1,steps)
    #     print temp.printBooms()
    # print '\n'
    # print s2.printBooms()
    G.build()
    # size = G.size()
    # for i, u in enumerate(G):
    #     # print u
    #     for e in G.adjcent(u):
    #         print e
