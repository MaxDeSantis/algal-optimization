
import enum
import math
from collections import namedtuple

from numpy import gradient

from algal_distribution import AlgalDistribution

class SearchBoat:
    class SearchState(enum.Enum):
        stop = 1,
        get_gradient = 2,
        line_search = 3,
        fixed_step_search = 4


    def __init__(self, algalDist, startX: float, startY: float):
        self.x = startX
        self.y = startY
        self.theta = 0 # radians
        self.velocity = 1 # meter per second
        self.state = self.SearchState.get_gradient
        self.concentration = None
        self.stepSize = 0.5 # Two meters for fixed step search
        self.samplingRadius = .5 # Distance of sampling
        self.sampleCount = 32
        self.fixedSearching = False
        self.steepestDescent = True

        self.algalDist = algalDist

        self.Measurement = namedtuple('Measurement', 'xPos yPos angle val')
    
        
    def GetDistance(self, x1, y1, x2, y2):
        d1 = (x1-x2)**2
        d2 = (y1 - y2)**2
        d = math.sqrt(d1 + d2)
        return d


        
    # Sample values at n locations around starting location.
    def GetGradientBehavior(self):

        angleStep = 2*math.pi/self.sampleCount
        measurements = []

        # Gather samples
        for i in range(0, self.sampleCount):
            sampleAngle = (self.theta + (i * angleStep)) % (math.pi * 2)
            sampleX = self.x + math.cos(sampleAngle) * self.samplingRadius
            sampleY = self.y + math.sin(sampleAngle) * self.samplingRadius

            sampleVal = self.algalDist.GetConcentration(sampleX, sampleY)

            newMeas = self.Measurement(sampleX, sampleY, sampleAngle, sampleVal)

            measurements.append(newMeas)

        # Determine maximum gradient (climbing)
        thetaBest = measurements[0].angle
        gradientMax = measurements[0].val - self.concentration

        for m in measurements:
            gradient = m.val - self.concentration

            if gradient > gradientMax:
                gradientMax = gradient
                thetaBest = m.angle

        # Stop if at maximum
        if gradientMax < 0:
            self.state = self.SearchState.stop
            return

        
        if self.steepestDescent:
            self.state = self.SearchState.line_search
        else:
            self.state = self.SearchState.fixed_step_search

        self.nextTheta = thetaBest
        self.nextX = self.x
        self.nextY = self.y
        
    # Continue until a set distance has been traversed, then recompute gradient
    def FixedSearchBehavior(self):

        # Handle behavior init
        if not self.fixedSearching:
            self.fixedSearching = True
            self.fixedSearchDistCovered = 0.0
        
        # Move to gradient search if have sufficient distance covered
        if self.fixedSearchDistCovered > self.stepSize:
            self.state = self.SearchState.get_gradient
            self.fixedSearching = False
        else:
            self.nextX = self.x + (self.velocity * self.deltaT) * math.cos(self.theta)
            self.nextY = self.y + (self.velocity * self.deltaT) * math.sin(self.theta)
            self.fixedSearchDistCovered += self.GetDistance(self.x, self.y, self.nextX, self.nextY)
    
    # Continue until a a minimum along line of travel has been reached, then recompute gradient.
    def LineSearchBehavior(self):
        if self.concentration > self.pastConcentration:
            self.state = self.SearchState.get_gradient
        else:
            self.nextX = self.x + (self.velocity * self.deltaT) * math.cos(self.theta)
            self.nextY = self.y + (self.velocity * self.deltaT) * math.sin(self.theta)

    def RunStateMachine(self, concentration: float, deltaT: float):
        self.pastConcentration = self.concentration
        self.concentration = concentration
        self.deltaT = deltaT

        if self.state == self.SearchState.stop:
            pass
        elif self.state == self.SearchState.get_gradient:
            self.GetGradientBehavior()
        elif self.state == self.SearchState.line_search:
            self.LineSearchBehavior()
        elif self.state == self.SearchState.fixed_step_search:
            self.FixedSearchBehavior()
        else:
            print("Unknown state")