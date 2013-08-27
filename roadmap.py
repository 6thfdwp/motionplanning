
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
        return self.state.distance(goal.state)

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
        self.fhandler.write("\n building road map with radius %.3f...\n" % self.radius)
        with Timer() as t:
            for i in range(N):
                # generate a valid sample state
                state = sampler.sampling() 
                self.addVertex( Vertex(state) )
        print 'total samples: %d, time cost: %.2f ms' % (self.size(), t.msecs)
        self.fhandler.write('total samples: %d, time cost: %.2f ms\n' % (self.size(), t.msecs) )
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
                    if u.state.distance(dest.state) > radius:
                        continue
                    if self.reachable(u.state, dest.state):
                        edges += 1
                        self.addEdge(u, dest)
        print 'total edges: %d' % edges
        self.fhandler.write('total edges: %d\n' % edges)
        print 'connect trys: %d, total connecting time: %.2f s' % (conn, t.secs)
        self.fhandler.write('connect trys: %d, total connecting time: %.2f s\n' % (conn, t.secs) ) 
        print 'generated intermediate states: %d' % (self.tempstateNum)
        self.fhandler.write('generated intermediate states: %d, time cost: %.2f s\n' % (self.tempstateNum, self.tempstateTime) )

    def incremental_reachable(self, source, dest, radius):
        reachable = True
        reachable_steps = 0
        dist = source.state.distance(dest.state)
        if dist > radius:
            return False
        with Timer() as t:
            stepNum = int(math.ceil(dist / 0.001) )
            for i in range(stepNum):
                tempState = self.sampler.interpolate(source.state, dest.state, i+1)
                self.tempstateNum += 1
                if self.isCollison(tempState):
                    reachable = False
                    break
        self.tempstateTime += t.secs
        # if reachable: 
        # reachable_steps = stepNum
        return reachable

    def reachable(self, source, dest):
        try:
            dist = source.distance(dest)
            stepNum = int(math.ceil(dist / 0.001) )
            if stepNum < 2 or dist < 0.001:
                return True
            t = stepNum / 2 
            # middle state in between
            # tempState = self.sampler.interpolate(source, dest, t)
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

    def addEdge(self, source, dest):
        newedge = Edge(source, dest)
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
    coords = [(0.2,0.3), (0.4,0.3), (0.4,0.5), (0.2,0.5)]
    obs = Obstacle(coords)
    init = State([(0.185,0.240), (0.150,0.180), (0.220,0.180)])
    G = RoadMap(asvNum=3, obstacles=[obs], N=200, radius=0.15)
    G.addVertex( Vertex(init) )
    # print G.N, G.radius
    sam = Sampler(G)
    s1 = sam.sampling()
    s2 = sam.sampling()
    s3 = sam.sampling()
    dist = s1.distance(s2)
    steps = dist / 0.001
    i = steps / 2
    print 'distance: %.3f' % dist
    print s1.printBooms()
    print '\n'

    temp = sam.interpolate_adv(s1,s2,i,steps)
    print temp.printBooms()
    temp = sam.interpolate_adv(s1,s2,i/2,steps)
    print temp.printBooms()
    # for i in range(5):
    #     temp = sam.interpolate_adv(s1,s2,i,steps)
    #     # print temp.points
    #     print temp.printBooms()
    print '\n'
    print s2.printBooms()
    # print "distance"
    # G.build()
    # size = G.size()
    # for i, u in enumerate(G):
    #     # print u
    #     for e in G.adjcent(u):
    #         print e
