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
                if ( Q[dest] > estCost ): # if lower cost found from other node 
                    Q[dest] = estCost # apply heuristic to decide which one need to be expanded next
                    dest.cost = cost
                    dest.setPredecessor(vert)
                    # print "pre:%s cost:%f" % (str(vert), cost)
        print 'no path found :('

    def getPath(self, dest):
       p = dest.getPredecessor()
       if p is None:
           return [str(dest)]
       return self.getPath(p) + [str(dest)]

    def writeOutput(self):
       for each in self.getPath(self.goal):
           self.fhandle.write(each)
           self.fhandle.write("\n")

if __name__ == '__main__':
    p = ASVPlanner(sys.argv[1], 50, 0.1)
    # print 'initState:' + str(p.init)
    # print 'goalState:' + str(p.goal)
