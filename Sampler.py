# from __future__ import print_function

import random, math, sys
from motionplanning.State import State
from shapely.geometry import *

class Sampler:
    def __init__(self, roadmap):
        self.asvNum = roadmap.asvNum
        self.BOOM_LENGTH = (0.050, 0.075)
        self.PRIMITIVE_STEP = 0.001
        n = self.asvNum - 1
        self.MIN_AREA = ( (0.007 * n) ** 2 ) * math.pi
        self.roadmap = roadmap
        self.narrowState = None
        self.samcount = 0
        # test only #
        # self.logfileHandle = open("sampling.log", "w")

    def _randomState(self, initPos=None):
        self.o = initPos or ( random.random(), random.random() )
        self.booms = []
        for i in range(self.asvNum -1):
            if i == 0:
                angle = (random.random()-0.5) * math.pi * 2
            else:
                angle = (random.random()) * math.pi
            length = random.uniform( self.BOOM_LENGTH[0], self.BOOM_LENGTH[1] )
            self.booms.append(( angle, length ))
        s = State( self.o, self.booms )
        return s

    def sampling(self):
        if self.narrowState:
            if self.samcount == 10:
                self.samcount = 0
                self.narrowState = None
                # print 'copied obs state'
            else:
                self.samcount += 1
                return self.copyNarrowState()
        while True:
            s = self._randomState()
            if s.isValid(self.MIN_AREA):
                if not self.roadmap.isCollison(s):
                    return s
                else:
                    if self.nearObsBounds(s):
                        narrowState = self.narrowSampling(s)
                        # print 'critical sample ',
                        # print narrowState
                        # return narrowState
                        if narrowState:
                            self.narrowState = narrowState
                            # print 'critical sample ',
                            # print narrowState
                            return narrowState
    
    def copyNarrowState(self):
        x, y = self.narrowState.o[0], self.narrowState.o[1]
        booms = self.narrowState.booms
        std = 0.08
        while True:
            newo = (random.gauss(x, std), random.gauss(y, std) )
            s = State(newo, list(booms) )
            if not self.roadmap.isCollison(s) and s.checkBoundary():
                # print s
                return s

    def nearObsBounds(self, refState):
        for i, obs in enumerate(self.roadmap.obstacles):
            if not obs.collide(refState): continue
            for line in obs.lines:
                if refState.shape().centroid.distance(line) < 0.05:
                    return True
        return False

    def narrowSampling(self, refState):
        x, y = refState.o[0], refState.o[1]
        std = 0.1
        for i in range(400):
        # while True:
            newo = (random.gauss(x, std), random.gauss(y, std) )
            s = self._randomState(newo)
            # if s.isValid(self.MIN_AREA) and not self.roadmap.isCollison(s):
            #     return s

            if s.isValid(self.MIN_AREA) and self.roadmap.isCollison(s):
                dist = refState.initDistance(s)
                # numSteps = int(math.ceil(dist / 0.001) )
                to = LineString([refState.o, s.o]).interpolate(dist/2)
                # print to
                # generate the middle state
                # midState = self.interpolate_adv(refState, s, numSteps/2, numSteps)
                midState = self._randomState( (to.x,to.y) )
                if not self.roadmap.isCollison(midState):
                    return midState
        return None

    # according to current step i, generate a state between source and dest
    def interpolate(self, source, dest, t):
        sx, sy = source.o
        dx, dy = dest.o
        dist = self.PRIMITIVE_STEP * t
        ipoint = LineString([(sx,sy), (dx,dy)]).interpolate(dist)
        newPos = (ipoint.x, ipoint.y)
        s = State(newPos, list(source.booms))
        return s
    
    def interpolate_adv(self, source, dest, t, numSteps):
        sx, sy = source.o
        dx, dy = dest.o
        steplen = t * source.initDistance(dest) / numSteps
        ipoint = LineString([(sx,sy), (dx,dy)]).interpolate(steplen)
        newPos = (ipoint.x, ipoint.y)
        newBooms = []
        for i, boom in enumerate(source.booms):
            sangle, slen = boom
            dangle, dlen = dest.booms[i]
            nl = slen + t*(dlen - slen) / numSteps
            nn = sangle + t*(dangle - sangle) / numSteps
            newBooms.append( (nn,nl) )

        s = State(newPos, newBooms)
        return s
