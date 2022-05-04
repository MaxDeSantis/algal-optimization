
from algal_distribution import AlgalDistribution
from boat_fsm import SearchBoat
import time
import math

import numpy as np
import matplotlib.pyplot as plt


class AlgalOptimizationSimulator:
    def __init__(self):
        self.boatStartingX = -7.0
        self.boatStartingY = 13.0

        self.meanX = 2.0
        self.meanY = -4.6
        self.cov = [[10, 0], [0, 10]]

        self.algalDist = AlgalDistribution(self.meanX, self.meanY, self.cov)
        self.boat = SearchBoat(self.algalDist, self.boatStartingX, self.boatStartingY)

        self.time = 0
        self.boat.theta = math.pi

        self.loopDuration = 0.5 # Seconds
        self.pauseDelay = 0.05 # seconds

        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        plt.autoscale(False)
        plt.ion()
        plt.contourf(self.algalDist.x, self.algalDist.y, self.algalDist.z, levels=20)
        plt.show()



    def PrintSimStatus(self):
        print('T:', self.time, 'STATE:', self.boat.state, 'CONCENTRATION:', self.boat.concentration, 'POS: ' + str(self.boat.x) + ',' + str(self.boat.y) + ')', "DISTANCE:", self.boat.GetDistance(self.boat.x, self.boat.y, self.meanX, self.meanY))

    def PlotSimStatus(self):
        

        plt.plot(self.boat.x, self.boat.y, marker="x", markersize=2, markerfacecolor='red', markeredgecolor='red')
        plt.draw()
        plt.pause(0.001)


    def Loop(self):
        while True:
            
            concentration = self.algalDist.GetConcentration(self.boat.x, self.boat.y)
            self.boat.RunStateMachine(concentration, deltaT=self.loopDuration)
            self.PrintSimStatus()

            # Move boat
            self.boat.x = self.boat.nextX
            self.boat.y = self.boat.nextY
            self.boat.theta = self.boat.nextTheta

            # Increment time
            self.PlotSimStatus()
            time.sleep(self.pauseDelay)
            self.time += self.loopDuration
            



sim = AlgalOptimizationSimulator()
sim.Loop()
