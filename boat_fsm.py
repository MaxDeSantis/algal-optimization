
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
    
    class SearchTechnique(enum.Enum):
        fixed_step = 1,
        steepest_descent = 2

    def __init__(self, algalDist: AlgalDistribution, startX: float, startY: float, deltaT: float, searchTechnique:SearchTechnique):
        self.x = startX
        self.y = startY
        self.theta = 0 # radians
        self.velocity = 1 # meter per second
        self.state = self.SearchState.get_gradient
        self.concentration = None
        self.stepSize = 3 # Two meters for fixed step search
        self.samplingRadius = 1.5 # Distance of sampling
        self.sampleCount = 32
        self.fixedSearching = False
        self.deltaT = deltaT
        self.searchTechnique = searchTechnique
        self.computingGradient = False

        self.algalDist = algalDist

        self.Measurement = namedtuple('Measurement', 'xPos yPos angle val')
    
    
    # Basic distance equation
    def GetDistance(self, x1, y1, x2, y2):
        d1 = (x1-x2)**2
        d2 = (y1 - y2)**2
        d = math.sqrt(d1 + d2)
        return d

    def GetAngle(self, x1, y1, x2, y2):
        ang = math.atan2(y2-y1, x2-x1)
        return ang

    def TravelToPoint(self, pointX: float, pointY: float) -> bool:
        # Orient towards point

        self.theta = self.GetAngle(self.x, self.y, pointX, pointY)

        # Check if point is beyond our maximum movable distance
        dist = self.GetDistance(pointX, pointY, self.x, self.y) 
        if dist > (self.velocity * self.deltaT):
            # Travel as far as possible
            self.nextX = self.x + (self.velocity * self.deltaT) * math.cos(self.theta)
            self.nextY = self.y + (self.velocity * self.deltaT) * math.sin(self.theta)
            self.nextTheta = self.theta
            return False # Point not reached
        else:
            # Point is closer than maximum travel, don't travel too far.
            self.nextX = self.x + dist * math.cos(self.theta)
            self.nextY = self.y + dist * math.sin(self.theta)
            self.nextTheta = self.theta
            return True # point reached

    # Sample values at n locations around starting location.
    def GetGradientBehavior(self):
        # Init
        if not self.computingGradient:
            self.angleStep = 2*math.pi/self.sampleCount
            self.measurements = []
            self.computingGradient = True
            self.gradientStep = 0
            self.startTheta = self.theta
            self.gradientCenterX = self.x
            self.gradientCenterY = self.y
            self.gradientCenterConcentration = self.concentration

        # Get sample point position
        sampleAngle = (self.startTheta + (self.gradientStep * self.angleStep)) % (math.pi * 2)
        sampleX = self.gradientCenterX + math.cos(sampleAngle) * self.samplingRadius
        sampleY = self.gradientCenterY + math.sin(sampleAngle) * self.samplingRadius
        
        if self.gradientStep == self.sampleCount:
            destReached = self.TravelToPoint(self.gradientCenterX, self.gradientCenterY)
        else:
            destReached = self.TravelToPoint(sampleX, sampleY)


        if destReached:
            # Gather sample, move to next sample
            if self.gradientStep < self.sampleCount:
                # Take sample
                sampleVal = self.algalDist.GetConcentration(sampleX, sampleY)
                newMeas = self.Measurement(sampleX, sampleY, sampleAngle, sampleVal)
                self.measurements.append(newMeas)
            self.gradientStep += 1


        # Check if completed search
        if self.gradientStep > self.sampleCount:
            self.computingGradient = False

            thetaBest = self.measurements[0].angle
            gradientMax = self.measurements[0].val - self.gradientCenterConcentration

            for m in self.measurements:
                gradient = m.val - self.gradientCenterConcentration

                if gradient < gradientMax:
                    gradientMax = gradient
                    thetaBest = m.angle

            # Stop if all feasible directions have positive gradients: at minimum
            if gradientMax > 0:
                self.state = self.SearchState.stop
                print("FOUND")
                return
            
            self.nextTheta = thetaBest
            # Transition to next state
            if self.searchTechnique == self.SearchTechnique.steepest_descent:
                self.state = self.SearchState.line_search
            else:
                self.state = self.SearchState.fixed_step_search

        # # Update boat state
        # self.nextTheta = thetaBest
        # self.nextX = self.x
        # self.nextY = self.y
        
    # Continue until a set distance has been traversed, then recompute gradient
    def FixedSearchBehavior(self):

        if not self.fixedSearching:
            self.fixedSearching = True
            self.fixedX = self.x + self.stepSize * math.cos(self.theta)
            self.fixedY = self.y + self.stepSize * math.sin(self.theta)
        
        destReached = self.TravelToPoint(self.fixedX, self.fixedY)

        if destReached:
            self.state = self.SearchState.get_gradient
            self.fixedSearching = False

        
        # # Move to gradient search if have sufficient distance covered
        # if self.fixedSearchDistCovered > self.stepSize:
        #     self.state = self.SearchState.get_gradient
        #     self.fixedSearching = False
        # else:
        #     self.nextX = self.x + (self.velocity * self.deltaT) * math.cos(self.theta)
        #     self.nextY = self.y + (self.velocity * self.deltaT) * math.sin(self.theta)
        #     self.fixedSearchDistCovered += self.GetDistance(self.x, self.y, self.nextX, self.nextY)
    
    # Continue until a a minimum along line of travel has been reached, then recompute gradient.
    def LineSearchBehavior(self):
        # Stop line search if concentration started increasing
        if self.concentration > self.pastConcentration:
            self.state = self.SearchState.get_gradient
        else:
            self.nextX = self.x + (self.velocity * self.deltaT) * math.cos(self.theta)
            self.nextY = self.y + (self.velocity * self.deltaT) * math.sin(self.theta)

    def RunStateMachine(self, concentration: float):
        self.pastConcentration = self.concentration
        self.concentration = concentration

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