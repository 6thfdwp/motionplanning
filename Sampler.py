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
        # test only #
        # self.logfileHandle = open("sampling.log", "w")

    def sampling(self):
        while True:
            self.o = ( random.random(), random.random() )
            self.booms = []
            for i in range(self.asvNum -1):
                angle = (random.random()) * math.pi
                length = random.uniform( self.BOOM_LENGTH[0], self.BOOM_LENGTH[1] )
                self.booms.append(( angle, length ))

            s = State( self.o, self.booms )
            if s.isValid(self.MIN_AREA) and not self.roadmap.isCollison(s):
                break
        # if not s.isValid() or self.roadmap.isCollison(s):
        #     return self.sampling()
        # print (str(s), file=self.logfileHandle)
        return s
    
    def verify(state):
        pass

    def booms(self):
        # self.booms = []
        for i in range(self.asvNum -1):
            angle = (random.random()) * math.pi
            length = random.uniform( self.BOOM_LENGTH[0], self.BOOM_LENGTH[1] )
            self.booms.append(( angle, length ))
        return booms

    # according to current step i, generate a state between source and dest
    def interpolate(self, source, dest, t):
        sx, sy = source.o
        dx, dy = dest.o
        dist = self.PRIMITIVE_STEP * t
        ipoint = LineString([(sx,sy), (dx,dy)]).interpolate(dist)
        newPos = (ipoint.x, ipoint.y)
        if not source.booms: # for the initial and goal with no angle and length info 
            booms = dest.booms
        else:
            booms = source.booms
        s = State(newPos, list(booms))
        # print str(s)
        return s
    
    def interpolate_adv(self, source, dest, t, numSteps):
        sx, sy = source.o
        dx, dy = dest.o
        dist = self.PRIMITIVE_STEP * t # the t-th step
        # r = t / numSteps
        ipoint = LineString([(sx,sy), (dx,dy)]).interpolate(dist)
        newPos = (ipoint.x, ipoint.y)
        newBooms = []
        initangle = 0.0
        booms = source.booms or dest.booms
        # if not source.booms:
        #     booms = dest.booms
        # else:
        #     booms = source.booms
        for i, boom in enumerate(booms):
            sangle = initangle or boom[0]
            dangle = dest.booms[i][0]
            # angle_dist = dest.booms[i][0] - boom[0]
            length = random.uniform( self.BOOM_LENGTH[0], self.BOOM_LENGTH[1] )
            nn = sangle + t*(dangle - sangle) / numSteps
            newBooms.append( (nn,length) )

        s = State(newPos, newBooms)
        return s
