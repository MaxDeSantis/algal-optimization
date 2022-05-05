
from algal_distribution import AlgalDistribution
from boat_fsm import SearchBoat
import time
import math

import numpy as np
import matplotlib.pyplot as plt


class AlgalOptimizationSimulator:
    def __init__(self):
        self.boatSearchTechnique = SearchBoat.SearchTechnique.steepest_descent
        self.boatStartingX = -4.0
        self.boatStartingY = 17.0

        self.loopDuration = 0.5 # Seconds
        self.pauseDelay = 0.001 # seconds

        self.meanX = 4.0
        self.meanY = -4.6
        self.cov = [[10, 15], [15, 40]]

        self.algalDist = AlgalDistribution(self.meanX, self.meanY, self.cov)
        self.boat = SearchBoat(self.algalDist, self.boatStartingX, self.boatStartingY, self.loopDuration, self.boatSearchTechnique)
        
        self.time = 0
        self.boat.theta = math.pi

        

        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        plt.autoscale(False)
        plt.ion()
        plt.contourf(self.algalDist.x, self.algalDist.y, self.algalDist.z, levels=20)
        plt.show()



    def PrintSimStatus(self):
        if not self.boat.state == self.boat.SearchState.stop:
            print('T:', self.time, 'STATE:', self.boat.state, 'CONCENTRATION:', self.boat.concentration, 'POS: ' + str(self.boat.x) + ',' + str(self.boat.y) + ')', "DISTANCE:", self.boat.GetDistance(self.boat.x, self.boat.y, self.meanX, self.meanY))
    def PlotSimStatus(self):
        plt.title(str(self.boatSearchTechnique) + f' T: {self.time:.2f} s')
        plt.xlabel('X Position, meters')
        plt.ylabel('Y Position, meters')
        plt.plot(self.boat.x, self.boat.y, marker="x", markersize=2, markerfacecolor='red', markeredgecolor='red')
        plt.draw()
        plt.pause(0.001)


    def Loop(self):
        while not self.boat.state == SearchBoat.SearchState.stop:
            
            concentration = self.algalDist.GetConcentration(self.boat.x, self.boat.y)
            self.boat.RunStateMachine(concentration)
            self.PrintSimStatus()

            # Move boat
            self.boat.x = self.boat.nextX
            self.boat.y = self.boat.nextY
            self.boat.theta = self.boat.nextTheta

            # Increment time
            self.PlotSimStatus()
            self.time += self.loopDuration

        plt.ioff()
        plt.show()
sim = AlgalOptimizationSimulator()
sim.Loop()

# sim.boat.TravelToPoint(0, 0)
# sim.boat.deltaT = sim.loopDuration
# sim.PlotSimStatus()