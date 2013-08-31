import sys, os
from motionplanning.roadmap import *

from motionplanning.Obstacle import Obstacle
from motionplanning.prioritydict import priority_dict

class ASVPlanner:
    def __init__(self, inputFile, sampleNum, connRadius):
        self.sampleNum = sampleNum
        self.connRadius = connRadius

        roadmap = self.setupProblem(inputFile)
        filename = 'output_' +str(sampleNum)+ '_' +str(connRadius)+ '_' +inputFile 
        self.fhandle = open(filename, "w")
        roadmap.build()
        print "degree of init: %d" % roadmap.degree(self.init)
        print "degree of goal: %d" % roadmap.degree(self.goal)
        print 'searching a path...'
        self.search()

    def setupProblem(self, inputFile):
        path = os.getcwd() + '/motionplanning/testcases/' + inputFile
        f = open(path)
        obstacles = []
        for i, line in enumerate(f):
            if i == 0: 
                asvNum = self.asvNum = int(line)
                continue
            if i == 1:
                self.initState = self.createState(line) 
                self.init = Vertex(self.initState)
                continue
            if i == 2:
                self.goalState = self.createState(line)
                self.goal = Vertex(self.goalState)
                continue
            if i > 3:
                obstacles.append( self.createObs(line) )
        G = self.roadmap = RoadMap(asvNum, obstacles, self.sampleNum, self.connRadius)
        G.addVertex( self.init )
        G.addVertex( self.goal )
        return G

    def createState(self, input):
        vals = input.split(' ')
        coords = []
        for i in xrange(0, len(vals), 2):
            if i / 2 == self.asvNum: break
            coords.append( ( float(vals[i]), float(vals[i+1]) ) )
        return State(coords)

    def createObs(self, input):
        vals = input.split(' ')
        coords = []
        for i in xrange(0, len(vals), 2):
            # if i / 2 == self.asvNum: break
            coords.append( ( float(vals[i]), float(vals[i+1]) ) )
        return Obstacle(coords)

    def search(self):
        G = self.roadmap
        Q = priority_dict()
        Q[self.init] = 0.0; visited = []
        self.init.cost = 0.0
        while Q:
            value, vert = Q.pop_smallest()
            if vert == self.goal:
                print('find a path!!!!')
                self.writeOutput()
                return
            visited.append(vert)
            for e in G.adjcent(vert): # explore neighbors
                dest = e.getDestination()
                if dest in visited:
                    continue
                # add new node to priority queue
                Q.setdefault(dest, float('inf')) 
                heuristic = dest.estimateCost(self.goal)
                cost = vert.cost + e.getWeight() 
                estCost = cost + heuristic
                if ( Q[dest] > estCost ): # if lower cost found
                    Q[dest] = estCost # use lower to increate priority
                    dest.cost = cost
                    dest.setPredecessor(vert)
                    # print "pre:%s cost:%f" % (str(vert), cost)
        print 'no path found :('

    def getPath(self, dest):
       p = dest.getPredecessor()
       if p is None:
           return [dest]
       return self.getPath(p) + [dest]

    def writeOutput(self):
       path = self.getPath(self.goal) 
       pathlen = len(path)
       for i, each in enumerate(path):
           if i == pathlen - 1: 
               break;
           source = path[i]
           dest = path[i+1]
           dist = source.state.distance(dest.state) 
           stepNum = int(math.ceil(dist / 0.001) )
           self.fhandle.write( str(source) + '\n')
           for i in range(stepNum):
               tempState = self.roadmap.sampler.interpolate_adv(source.state, dest.state, i+1, stepNum)
               self.fhandle.write(str(tempState) + '\n')

           self.fhandle.write(str(dest) + "\n")

if __name__ == '__main__':
    p = ASVPlanner(sys.argv[1], 50, 0.1)
    # print 'initState:' + str(p.init)
    # print 'goalState:' + str(p.goal)
